#include "../network/network.hpp"
#include "../shared.hpp"

#include "camera_feed.hpp"
#include "controller.hpp"
#include "log_view.hpp"
#include "debug_console.hpp"
#include "gui.hpp"
#include "log.hpp"

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

// Send movement updates 3x per second.
const int MOVEMENT_SEND_INTERVAL = 1000 / 3;
// Heartbeat interval
const int HEARTBEAT_SEND_INTERVAL = 1000 / 3;
const int RECONNECT_INTERVAL = 1000 / 3;
// Amount of time between heartbeats until disconnection flag is set
const int DISCONNECT_TIMER = 5000;

// Network connection.
network::Connection conn;

unsigned int map_texture_id;

// Save the start time so we can use get_ticks.
std::chrono::high_resolution_clock::time_point start_time;

unsigned int get_ticks()
{
    auto now = std::chrono::high_resolution_clock::now();

    return (unsigned int)std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
}

std::vector<std::string> split_by_spaces(std::string s) {
	std::vector<std::string> strings;

	size_t last = 0; 
	size_t next = 0; 

	while ((next = s.find(" ", last)) != std::string::npos) { 
		strings.push_back(s.substr(last, next - last));
		last = next + 1;
	}

	strings.push_back(s.substr(last));

	return strings;
}

network::MovementMessage last_movement_message = { 0, 0 };

void command_callback(std::string command) {
	auto parts = split_by_spaces(command);

	if (parts.size() == 0) {
		return;
	}

	if (parts[0] == "move") {
		if (parts.size() != 3) {
			return;
		}

		char left_direction_char = parts[1][0];
		int16_t left_speed = (int16_t) atoi(parts[1].substr(1).c_str());

		char right_direction_char = parts[2][0];
		int16_t right_speed = (int16_t) atoi(parts[2].substr(1).c_str());

		last_movement_message.left = left_speed;
		last_movement_message.right = right_speed;

		if (left_direction_char == 'b') {
			last_movement_message.left *= -1;
		}

		if (right_direction_char == 'b') {
			last_movement_message.right *= -1;
		}

		log::log(log::DEBUG, "> Update movement to %d, %d", last_movement_message.left, last_movement_message.right);
	}
}

void stderr_handler(log::Level level, std::string message) {
	fprintf(stderr, "%s\n", message.c_str());
}

void log_view_handler(log::Level level, std::string message) {
	float r, g, b, a = 1.0f;

	switch (level) {
	case log::DEBUG:
		r = 0.70f;
		g = 0.70f;
		b = 0.70f;
		break;
	case log::INFO:
		r = 1.0f;
		g = 1.0f;
		b = 1.0f;
		break;
	case log::WARNING:
		r = 1.00f;
		g = 0.52f;
		b = 0.01f;
		break;
	case log::ERROR:
		r = 1.0f;
		g = 0.0f;
		b = 0.0f;
		break;
	}

	gui::log_view::print(message, r, g, b, a);
}

struct Config
{
	int local_port;

	char remote_address[16]; // Max length of ipv4 address is 15, plus one for nt.
	int remote_port;
};

Config load_config(const char* filename) {
	Config config;

	FILE* file = fopen(filename, "r");
	if (!file) {
		log::log(log::ERROR, "Failed to open config file!");
		exit(1);
	}

	// First line: local_port remote_address remote_port
	fscanf(file, "%d %s %d\n", &config.local_port, config.remote_address, &config.remote_port);

	fclose(file);

	return config;
}

void do_gui(camera_feed::Feed feed[4], gui::Font *font)
{
    // Clear the screen to a modern dark gray.
    glClearColor(35.0f / 255.0f, 35.0f / 255.0f, 35.0f / 255.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    gui::Layout layout{};

    // Set margin.
    layout.advance_x(20);
    layout.advance_y(20);
    layout.push();

    // Draw the map.
    gui::do_textured_rect(&layout, 572, 572, map_texture_id);

    layout.reset_x();
    layout.advance_y(10);

    // Draw the log.
	int lvw = 572;
	int lvh = 458;
	gui::log_view::calc_sizing(font, lvw, lvh);
	gui::log_view::do_log(&layout, lvw, lvh, font);

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

    gui::do_solid_rect(&layout, 755, 300, 68.0f / 255.0f, 68.0f / 255.0f, 68.0f / 255.0f);

    // Draw the debug overlay.
    layout = {};
    gui::debug_console::do_debug(&layout, font);
}

void glfw_character_callback(GLFWwindow *window, unsigned int codepoint)
{
    if (gui::state.input_state == gui::InputState::DEBUG_CONSOLE) {
        if (codepoint < 128) {
            gui::debug_console::handle_input((char)codepoint);
        }
    }
}

void glfw_key_callback(GLFWwindow *window, int key, int scancode, int action, int mods)
{
    if (gui::state.input_state == gui::InputState::DEBUG_CONSOLE) {
        if (action == GLFW_PRESS || action == GLFW_REPEAT) {
            gui::debug_console::handle_keypress(key, mods);
        }
    }
}


int main()
{
	log::register_handler(stderr_handler);
	log::register_handler(log_view_handler);

	gui::debug_console::set_callback(command_callback);

	// Load config.
	Config config = load_config("res/bs_config.txt");

    // Start the timer.
    start_time = std::chrono::high_resolution_clock::now();

    // Init GLFW.
    if (!glfwInit()) {
		log::log(log::ERROR, "Failed to init GLFW!");
        return 1;
    }

    // Init the controller.
	// TODO: QUERY /sys/class/input/js1/device/id/{vendor,product} TO FIND THE RIGHT CONTROLLER.
    bool controller_loaded = false;
    if (controller::init("/dev/input/js1") == controller::Error::OK) {
        controller_loaded = true;
		log::log(log::INFO, "Controller connected.");
    } else {
		log::log(log::WARNING, "No controller connected!");
    }

    // Create a fullscreen window. Title isn't displayed, so doesn't really
    // matter.
    GLFWwindow *window =
        glfwCreateWindow(gui::WINDOW_WIDTH, gui::WINDOW_HEIGHT, "Base Station", glfwGetPrimaryMonitor(), NULL);

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
    camera_feed::Feed feeds[2];
    camera_feed::init_feed(&feeds[0], 1280, 720);
    camera_feed::init_feed(&feeds[1], 1280, 720);

    // Initialize network functionality.
    {
        network::Error err = network::connect(&conn, config.local_port, config.remote_address, config.remote_port);
        if (err != network::Error::OK) {
			log::log(log::ERROR, "Failed to connect to rover!");
            return 1;
        }
    }

    // Keep track of when we last sent movement and heartbeat info.
    unsigned int last_movement_send_time = 0;
    // Last time heartbeat was sent
    unsigned int last_heartbeat_send_time = 0;

    gui::Font font;
    bool loaded_font = gui::load_font(&font, "res/FiraMono-Regular.ttf", 100);

	map_texture_id = gui::load_texture("res/binghamton.jpg");

    if (!loaded_font) {
		log::log(log::ERROR, "Failed to load font!");
        return 1;
    }

    gui::debug_console::log("Debug log initialized.", 0, 1.0, 0);

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
            if (gui::state.input_state == gui::InputState::KEY_COMMAND) {
                gui::state.show_debug_console = true;
                gui::state.input_state = gui::InputState::DEBUG_CONSOLE;
            }
        } 

        // Handle incoming network messages.
        network::Message message;
        while (true) {
			network::Error neterr = network::poll(&conn, &message);
			if (neterr != network::Error::OK) {
				if (neterr == network::Error::NOMORE) {
					break;
				} else {
					log::log(log::WARNING, "Failed to read network packets!");
					break;
				}
			}

            switch (message.type) {
                case network::MessageType::HEARTBEAT: {
                    break;
                }
                case network::MessageType::CAMERA: {
                    // Static buffer so we don't have to allocate and reallocate every
                    // frame.
                    static uint8_t camera_message_buffer[CAMERA_MESSAGE_FRAME_DATA_MAX_SIZE];

                    network::CameraMessage camera_message;
                    camera_message.data = camera_message_buffer;
                    network::deserialize(message.buffer, &camera_message);

                    if (camera_message.stream_index > 2) {
                        break;
                    }

                    camera_feed::Error err = camera_feed::handle_section(
                        &feeds[camera_message.stream_index], camera_message.data, camera_message.size,
                        camera_message.section_index, camera_message.section_count, camera_message.frame_index);

                    if (err != camera_feed::Error::OK) {
						log::log(log::WARNING, "Failed to handle video frame section!");
                    }

                    break;
                }
                default:
                    break;
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

            // Do nothing since we just want to update current values.
            while ((err = controller::poll(&event)) == controller::Error::OK) {
				if (event.type == controller::EventType::AXIS) {
					bool forward = event.value <= 0;
					int16_t abs_val = event.value < 0 ? -event.value : event.value;

					log::log(log::DEBUG, "Got axis with %d", abs_val);

					if (event.axis == controller::Axis::JS_LEFT_Y) {
						last_movement_message.left = abs_val >> 7;
						if (!forward) last_movement_message.left *= -1;
					} else if (event.axis == controller::Axis::JS_RIGHT_Y) {
						last_movement_message.right = abs_val >> 7;
						if (!forward) last_movement_message.right *= -1;
					}
				}
			}

			if (err != controller::Error::DONE) {
				log::log(log::ERROR, "Failed to read from the controller! Disabling");
				controller_loaded = false;
			}
		}

		if (get_ticks() - last_movement_send_time >= MOVEMENT_SEND_INTERVAL) {
			last_movement_send_time = get_ticks();

			network::Buffer *message_buffer = network::get_outgoing_buffer();
			network::serialize(message_buffer, &last_movement_message);
			network::send(&conn, network::MessageType::MOVEMENT, message_buffer);
		}

        // Update and draw GUI.
        do_gui(feeds, &font);
        glColor4f(0.0f, 0.0f, 0.0f, 1.0f);

        // Display our buffer.
        glfwSwapBuffers(window);
    }

    // Cleanup.
    glfwTerminate();

    return 0;
}
