#include "../network/network.hpp"
#include "../shared.hpp"

#include "camera_feed.hpp"
#include "controller.hpp"
#include "debug_console.hpp"
#include "gui.hpp"
#include "logging_framework.h"

#include <GL/gl.h>
#include <GLFW/glfw3.h>

#include <cstdint>
#include <cstdio>
#include <cstdlib>

#include <chrono>
#include <iostream>
#include <stack>
#include <string>
#include <vector>

#include <iostream>
#include <string>
#include <vector>

#include <sys/types.h>
#include <unistd.h>

// Default angular resolution (vertices / radian) to use when drawing circles.
constexpr float ANGULAR_RES = 10.0f;

// We know that our base station will have this resolution.
const int WINDOW_WIDTH = 1920;
const int WINDOW_HEIGHT = 1080;

// Send bandwidth updates once per second
const int BANDWIDTH_SEND_INTERVAL = 1000;

// Send movement updates 3x per second.
const int MOVMENT_SEND_INTERVAL = 1000 / 3;
// Heartbeat interval
const int HEARTBEAT_SEND_INTERVAL = 1000 / 3;
const int RECONNECT_INTERVAL = 1000 / 3;
// Amount of time between heartbeats until disconnection flag is set
const int DISCONNECT_TIMER = 5000;

// List of messages the log displays
std::vector<std::string> logMessages;

// Stuff for the box of the log
const int LOG_X = 20;
const int LOG_Y = 600;
const int LOG_WIDTH = 400;
const int LOG_HEIGHT = 400;
const int LOG_THICKNESS = 2;
const unsigned int MAX_CHARS_IN_A_LINE = 32;
const unsigned int MAX_LINES = 15;

// Four parallel lists that are also parallel to logMessages
std::vector<float> red, green, blue, alpha;

// Save the start time so we can use get_ticks.
std::chrono::high_resolution_clock::time_point start_time;

unsigned int get_ticks() {
    auto now = std::chrono::high_resolution_clock::now();

    return (unsigned int)std::chrono::duration_cast<std::chrono::milliseconds>(
            now - start_time)
            .count();
}

void do_gui(camera_feed::Feed feed[4], gui::Font *font) {
    // Clear the screen to a modern dark gray.
    glClearColor(35.0f / 255.0f, 35.0f / 255.0f, 35.0f / 255.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    gui::Layout layout{};

    // Set margin.
    layout.advance_x(20);
    layout.advance_y(20);
    layout.push();

    // Draw the map.
    gui::do_solid_rect(&layout, 572, 572, 119.0f / 255.0f, 82.0f / 255.0f,
                       65.0f / 255.0f);

    layout.reset_x();
    layout.advance_y(10);

    // Draw the log.
    gui::do_solid_rect(&layout, 572, 458, 0, 0, 0);

    layout.reset_y();
    layout.advance_x(10);
    layout.push();

    // Draw the main camera feed.
    gui::do_textured_rect(&layout, 1298, 730, feed[0].gl_texture_id);

    layout.reset_x();
    layout.advance_y(10);
    layout.push();

    // Draw the other camera feed.
    layout.reset_y();
    gui::do_textured_rect(&layout, 533, 300, feed[1].gl_texture_id);

    layout.reset_y();
    layout.advance_x(10);

    gui::do_solid_rect(&layout, 755, 300, 68.0f / 255.0f, 68.0f / 255.0f,
                       68.0f / 255.0f);

    // Draw the debug overlay.
    layout = {};
    gui::debug_console::do_debug(&layout, font);
}

void glfw_character_callback(GLFWwindow *window, unsigned int codepoint) {
    if (gui::state.input_state == gui::InputState::DEBUG_CONSOLE) {
        if (codepoint < 128) {
            gui::debug_console::handle_input((char)codepoint);
        }
    }
}

void glfw_key_callback(GLFWwindow *window, int key, int scancode, int action,
                       int mods) {
    if (gui::state.input_state == gui::InputState::DEBUG_CONSOLE) {
        if (action == GLFW_PRESS || action == GLFW_REPEAT) {
            gui::debug_console::handle_keypress(key, mods);
        }
    }
}

// Helper method for addMessage
void removeOldMessages() {
    while (logMessages.size() > MAX_LINES) {
        logMessages.erase(logMessages.begin());
        red.erase(red.begin());
        green.erase(green.begin());
        blue.erase(blue.begin());
        alpha.erase(alpha.begin());
    }
}

// addMessage("Message Here", red, green, blue, alpha)
void addMessage(std::string m, float r, float g, float b, float a) {
    while (m.length() > 0) {
        if (m.length() <= MAX_CHARS_IN_A_LINE) {
            logMessages.push_back(m);
            red.push_back(r);
            green.push_back(g);
            blue.push_back(b);
            alpha.push_back(a);
            break;
        } else {
            logMessages.push_back(m.substr(0, MAX_CHARS_IN_A_LINE));
            red.push_back(r);
            green.push_back(g);
            blue.push_back(b);
            alpha.push_back(a);
            m.erase(0, MAX_CHARS_IN_A_LINE);
        }
    }
    removeOldMessages();
}

// Test method to make sure the log is working properly
void testLog() {
    addMessage("Hello World!", 1.0f, 1.0f, 1.0f, 1.0f);
    addMessage("Hello World!", 1.0f, 1.0f, 1.0f, 1.0f);
    addMessage("Hello World!", 1.0f, 1.0f, 1.0f, 1.0f);
    addMessage("Hello World!", 1.0f, 1.0f, 1.0f, 1.0f);
    addMessage("I'm BLUE da ba dee da ba die, da ba dee, da ba die, da ba dee da "
               "ba die!",
               0.0f, 0.0f, 1.0f, 1.0f);
    addMessage("Hello World!", 1.0f, 1.0f, 1.0f, 1.0f);
    addMessage("Error: The rover is literally on fire oh god oh geez oh no", 1.0f,
               0.0f, 0.0f, 1.0f);
    addMessage("Hello World!", 1.0f, 1.0f, 1.0f, 1.0f);
    addMessage("Hello World!", 1.0f, 1.0f, 1.0f, 1.0f);
    addMessage("Life: Exists", 0.0f, 1.0f, 1.0f, 1.0f);
    addMessage("Greek Philosophers: HmmmmMMMMMMMMmm", 0.0f, 1.0f, 1.0f, 1.0f);
    addMessage("Hello World!", 1.0f, 1.0f, 1.0f, 1.0f);
}

int main() {
    // Creating the logging framework
    logging_framework *log = new logging_framework();
    log->adjustLogLevel(INFO_LOG_LEVEL);
    log->callPrint("Information: This is the BU Mars Rover\n");

    // Start the timer.
    start_time = std::chrono::high_resolution_clock::now();

    // Init GLFW.
    if (!glfwInit()) {
        GLenum temp = glGetError();
        log->adjustLogLevel(ERROR_LOG_LEVEL);
        log->callPrint("[!] Error: Failed to init GLFW: ");
        log->callPrint((char *)glGetString(temp));
        log->callPrint(". Exiting program\n");
        return 1;
    }

    // Init the controller.
    bool controller_loaded = false;
    if (controller::init("/dev/input/js0") == controller::Error::OK) {
        controller_loaded = true;
    } else {
        log->adjustLogLevel(WARN_LOG_LEVEL);
        log->callPrint("Warning: No controller detected\n");
    }

    // Fill the Log with test messages
    testLog();

    // Create a fullscreen window. Title isn't displayed, so doesn't really
    // matter.
    GLFWwindow *window =
            glfwCreateWindow(gui::WINDOW_WIDTH, gui::WINDOW_HEIGHT, "Base Station",
                             glfwGetPrimaryMonitor(), NULL);

    // Update the window so everyone can access it.
    gui::state.window = window;

    // Set sticky keys mode. It makes our input work as intended.
    glfwSetInputMode(window, GLFW_STICKY_KEYS, 1);

    glfwSetCharCallback(window, glfw_character_callback);
    glfwSetKeyCallback(window, glfw_key_callback);

    // Create an OpenGL context.
    glfwMakeContextCurrent(window);

    // OpenGL Setup.
    glViewport(0, 0, gui::WINDOW_WIDTH, gui::WINDOW_HEIGHT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, gui::WINDOW_WIDTH, gui::WINDOW_HEIGHT, 0, 0, 0.5);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Initialize camera stuff.
    camera_feed::init();

    // Create the camera streams.
    camera_feed::Feed feeds[4];
    camera_feed::init_feed(&feeds[0], 1920, 1080);
    camera_feed::init_feed(&feeds[1], 1920, 1080);
    camera_feed::init_feed(&feeds[2], 1920, 1080);
    camera_feed::init_feed(&feeds[3], 1920, 1080);

    network::Error connection_status = network::Error::DISCONNECT;
    // Initialize network functionality.
    network::Connection conn;
    {
        network::Error err = network::connect(&conn, "127.0.0.1", 45546, 45545);
        if (err != network::Error::OK) {
            // Sending error report to logging framework
            log->adjustLogLevel(ERROR_LOG_LEVEL);
            log->callPrint("[!]Error: Failed to connect to rover. Exiting program\n");
            return 1;
        }
        connection_status = network::Error::OK;
    }

    log->adjustLogLevel(TRACE_LOG_LEVEL);

    // Keep track of when we last sent movement and heartbeat info.
    unsigned int last_movement_send_time = 0;
    // Last time heartbeat was sent
    unsigned int last_heartbeat_send_time = 0;
    // Last time heartbeat was recieved
    unsigned int last_heart_received = 0;
    // Last time reconection was attempted
    unsigned int last_reconnect_attempt = 0;

    unsigned int last_bandwidth_send_time = 0;
    unsigned int total_bytes = 0;
    double current_bandwidth = 0;
    unsigned int bandwidth_time_passed = 0;
    double round_trip_time = 0;

    gui::Font debug_console_font;
    bool loaded_font =
            gui::load_font(&debug_console_font, "res/FiraMono-Regular.ttf", 100);
    if (!loaded_font) {
        fprintf(stderr, "[!] Failed to load debug console font!\n");
        return 1;
    }

    gui::debug_console::log("Debug log initialized.", 0, 1.0, 0);

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
            if (!gui::state.show_debug_console) {
                gui::state.show_debug_console = true;
                gui::state.input_state = gui::InputState::DEBUG_CONSOLE;
            }
        } else if (glfwGetKey(window, GLFW_KEY_TAB) == GLFW_PRESS) {
            if (!log->getDebugStatus()) {
                log->setDebugStatus(true);
                log->setDebugMessageStatus(false);
            } else {
                log->setDebugStatus(false);
                log->setDebugMessageStatus(false);
            }
        }

        // condition for turning on and off debug mode
        if (log->getDebugStatus() && !log->getDebugMessageStatus()) {
            log->adjustLogLevel(DEBUG_LOG_LEVEL);
            log->callPrint("Debug mode has been turned on\n");
            log->setDebugMessageStatus(true);
        }
        if (!log->getDebugStatus() && !log->getDebugMessageStatus()) {
            log->callPrint("Debug mode has been turned off\n");
            log->adjustLogLevel(TRACE_LOG_LEVEL);
            log->setDebugMessageStatus(true);
        }

        // Check connection status
        network::Error connection_status = network::check_status(&conn);
        if (connection_status != network::Error::OK) {
            fprintf(stderr, "[!] The socket was closed\n");
        }

        // Handle network missing heartbeat disconnect
        if (get_ticks() - last_heart_received >= DISCONNECT_TIMER) {
            last_heart_received = get_ticks();
            fprintf(stderr,
                    "[!] Too much time has passed since the last heartbeat\n");
        }

        if (connection_status == network::Error::DISCONNECT &&
            last_reconnect_attempt > RECONNECT_INTERVAL) {
            last_reconnect_attempt = get_ticks();
            network::Error reconnect =
                    network::reconnect(&conn, "127.0.0.1", 45546, 45545);
            if (reconnect != network::Error::OK) {
                fprintf(stderr, "[!] Failed to reconnect\n");
            } else {
                fprintf(stderr, "[!] Reconnection succeeded\n");
                connection_status = network::Error::OK;
            }
        }

        // Handle incoming network messages.
        network::poll_incoming(&conn);

        network::Message message;
        // network::Buffer* outgo = network::get_outgoing_buffer();
        // network::queue_outgoing(&conn, network::MessageType::HEARTBEAT, outgo);

        while (network::dequeue_incoming(&conn, &message)) {
            total_bytes = sizeof(message);
            bandwidth_time_passed = get_ticks();
            switch (message.type) {
                case network::MessageType::HEARTBEAT: {
                    printf("Recieved a heartbeat response from rover\n");
                    last_heart_received = get_ticks();
                    break;
                }
                case network::MessageType::CAMERA: {
                    // Static buffer so we don't have to allocate and reallocate every
                    // frame.
                    static uint8_t
                            camera_message_buffer[CAMERA_MESSAGE_FRAME_DATA_MAX_SIZE];

                    network::CameraMessage camera_message;
                    camera_message.data = camera_message_buffer;
                    network::deserialize(message.buffer, &camera_message);

                    if (camera_message.stream_index > 4) {
                        break;
                    }

                    camera_feed::Error err = camera_feed::handle_section(
                            &feeds[camera_message.stream_index], camera_message.data,
                            camera_message.size, camera_message.section_index,
                            camera_message.section_count, camera_message.frame_index);
                    if (err != camera_feed::Error::OK) {
                        // sending error message to log
                        log->adjustLogLevel(ERROR_LOG_LEVEL);
                        log->callPrint("[!] Error: Failed to handle frame section\n");
                        log->adjustLogLevel(TRACE_LOG_LEVEL);
                    }

                    break;
                }
                default:
                    break;
            }

            network::return_incoming_buffer(message.buffer);
            if (get_ticks() - last_bandwidth_send_time > 0) {
                last_bandwidth_send_time = get_ticks();
                current_bandwidth =
                        total_bytes / (last_bandwidth_send_time - bandwidth_time_passed);
                printf("%f bandwidth\n", current_bandwidth);
            }

            network::return_incoming_buffer(message.buffer);
        }
        // Reset heartbeat send time
        if (get_ticks() - last_heartbeat_send_time >= HEARTBEAT_SEND_INTERVAL) {
            last_heartbeat_send_time = get_ticks();
        }

        if (controller_loaded) {
            // Process controller input.
            controller::Event event;
            controller::Error err;
            double time_passed;

            // Do nothing since we just want to update current values.
            while ((err = controller::poll(&event)) == controller::Error::OK) {
            }

            if (err != controller::Error::DONE) {
                log->adjustLogLevel(ERROR_LOG_LEVEL);
                log->callPrint(
                        "[!] Error: Failed to read from the controller. Disabling\n");
                log->adjustLogLevel(TRACE_LOG_LEVEL);
                controller_loaded = false;
            } else {
                if (get_ticks() - last_movement_send_time >= MOVMENT_SEND_INTERVAL) {
                    bandwidth_time_passed = get_ticks() - last_movement_send_time;
                    round_trip_time = bandwidth_time_passed / 1000;

                    last_movement_send_time = get_ticks();
                    fprintf(stderr, "[!] Sending controller data\n");
                    network::Buffer *message_buffer = network::get_outgoing_buffer();
                    network::MovementMessage message;
                    message.left = -controller::get_value(controller::Axis::JS_LEFT_Y);
                    message.right = -controller::get_value(controller::Axis::JS_RIGHT_Y);
                    network::serialize(message_buffer, &message);

                    network::queue_outgoing(&conn, network::MessageType::MOVEMENT,
                                            message_buffer);
                    time_passed = 1000 / bandwidth_time_passed;
                    current_bandwidth = sizeof(message) * time_passed;
                    printf("%f bandwidth in bytes per second \n", current_bandwidth);
                    printf("%f round trip time in ms \n", round_trip_time);
                }
            }
        }

        // Update and draw GUI.
        do_gui(feeds, &debug_console_font);
        glColor4f(0.0f, 0.0f, 0.0f, 1.0f);

        // Init the font
        gui::Font font;
        gui::load_font(&font, "res/F25_Bank_Printer.ttf", 100);

        // Draw log text now
        for (unsigned int i = 0; i < logMessages.size(); i++) {
            const char *cstr = logMessages.at(i).c_str();
            glColor4f(red.at(i), green.at(i), blue.at(i), alpha.at(i));
            gui::draw_text(&font, cstr, LOG_X + LOG_THICKNESS + 5,
                           LOG_Y + LOG_THICKNESS + 5 + 30 * i, 20);
        }

        // Display our buffer.
        glfwSwapBuffers(window);

        // Send any messages that we accumulated.
        fprintf(stderr, "[!] Draining queue.\n");
        network::drain_outgoing(&conn);
    }

    // Cleanup.
    glfwTerminate();

    return 0;
}
