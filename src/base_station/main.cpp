#include "controller.hpp"
#include "camera_feed.hpp"
#include "gui.hpp"

#include "../network/network.hpp"
#include "../shared.hpp"

#include <GLFW/glfw3.h>
#include <GL/gl.h>

#include <cstdio>
#include <cstdlib>
#include <cstdint>

#include <stack>
#include <chrono>

// Default angular resolution (vertices / radian) to use when drawing circles.
constexpr float ANGULAR_RES = 10.0f;

// We know that our base station will have this resolution.
const int WINDOW_WIDTH = 1920;
const int WINDOW_HEIGHT = 1080;

// Send movement updates 3x per second.
const int MOVMENT_SEND_INTERVAL = 1000/3;

// Send bandwidth updates once per second
const int BANDWIDTH_SEND_INTERVAL = 1000;

// Save the start time so we can use get_ticks.
std::chrono::high_resolution_clock::time_point start_time;

unsigned int get_ticks() {
	auto now = std::chrono::high_resolution_clock::now();

	return (unsigned int) std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
}

void do_gui(camera_feed::Feed feed[4]) {
    // Clear the screen to a modern dark gray.
    glClearColor(35.0f / 255.0f, 35.0f / 255.0f, 35.0f / 255.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    gui::Layout layout{};

    // Set margin.
    layout.advance_x(20);
    layout.advance_y(20);
    layout.push();

    // Draw the map.
    gui::do_solid_rect(&layout, 572, 572, 119.0f / 255.0f, 82.0f / 255.0f, 65.0f / 255.0f);

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

	gui::do_solid_rect(&layout, 755, 300, 68.0f / 255.0f, 68.0f / 255.0f, 68.0f / 255.0f);
}

int main() {
	// Start the timer.
	start_time = std::chrono::high_resolution_clock::now();

	// Init GLFW.
    if (!glfwInit()) {
        fprintf(stderr, "[!] Failed to init GLFW!\n");
        return 1;
    }

    // Init the controller.
    bool controller_loaded = false;
    if (controller::init("/dev/input/js0") == controller::Error::OK) {
        controller_loaded = true;
    } else {
        printf("No controller.\n");
    }

    // Create a fullscreen window. Title isn't displayed, so doesn't really matter.
	GLFWwindow* window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Base Station", glfwGetPrimaryMonitor(), NULL);

	// Set sticky keys mode. It makes our input work as intended.
	glfwSetInputMode(window, GLFW_STICKY_KEYS, 1);

    // Create an OpenGL context.
	glfwMakeContextCurrent(window);

    // OpenGL Setup.
    glViewport(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, WINDOW_WIDTH, WINDOW_HEIGHT, 0, 0, 0.5);

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

    // Initialize network functionality.
    network::Connection conn;
    {
        network::Error err = network::connect(&conn, "127.0.0.1", 45546, 45545);
        if (err != network::Error::OK) {
            fprintf(stderr, "[!] Failed to connect to rover!\n");
            return 1;
        }
    }

    // Keep track of when we last sent movement info.
    unsigned int last_movement_send_time = 0;
    unsigned int last_bandwidth_send_time = 0;
    unsigned int total_bytes = 0;
    double current_bandwidth = 0;
    unsigned int bandwidth_time_passed = 0;
    double round_trip_time = 0;

    while (!glfwWindowShouldClose(window)) {
		glfwPollEvents();

		if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
			break;
		}

        // Handle incoming network messages.
        network::poll_incoming(&conn);
        
        network::Message message;

	while (network::dequeue_incoming(&conn, &message)) {
		total_bytes = sizeof(message);
		bandwidth_time_passed = get_ticks();
		switch (message.type) {
			case network::MessageType::HEARTBEAT: {
                   network::Buffer* outgoing = network::get_outgoing_buffer();
                    network::queue_outgoing(&conn, network::MessageType::HEARTBEAT, outgoing);
                    break;
                }
                case network::MessageType::CAMERA: {
                    // Static buffer so we don't have to allocate and reallocate every frame.
                    static uint8_t camera_message_buffer[CAMERA_MESSAGE_FRAME_DATA_MAX_SIZE];
                    network::CameraMessage camera_message;
                    camera_message.data = camera_message_buffer;
                    network::deserialize(message.buffer, &camera_message);

                    if (camera_message.stream_index > 4) {
                        break;
                    }

                    camera_feed::Error err = camera_feed::handle_section(&feeds[camera_message.stream_index], camera_message.data, camera_message.size, camera_message.section_index, camera_message.section_count, camera_message.frame_index);
                    if (err != camera_feed::Error::OK) {
                        fprintf(stderr, "[!] Failed to handle frame section!\n");
                    }

                    break;
                }
                default:
                    break;
		}
		network::return_incoming_buffer(message.buffer);
		if(get_ticks() - last_bandwidth_send_time > 0){
			last_bandwidth_send_time = get_ticks();
			current_bandwidth = total_bytes / (last_bandwidth_send_time - bandwidth_time_passed);
			printf("%f bandwidth\n",current_bandwidth);
		}	       


        }

        if (controller_loaded) {
            // Process controller input.
            controller::Event event;
            controller::Error err;
            double time_passed;
            // Do nothing since we just want to update current values.
            while ((err = controller::poll(&event)) == controller::Error::OK) {}

            if (err != controller::Error::DONE) {
                fprintf(stderr, "[!] Failed to read from the controller! Disabling.\n");
                controller_loaded = false;
            } else {
                if (get_ticks() - last_movement_send_time >= MOVMENT_SEND_INTERVAL) {
		    bandwidth_time_passed = get_ticks() - last_movement_send_time;
		    round_trip_time = bandwidth_time_passed/1000;
                    last_movement_send_time = get_ticks();

                    network::Buffer* message_buffer = network::get_outgoing_buffer();

                    network::MovementMessage message;
                    message.left = -controller::get_value(controller::Axis::JS_LEFT_Y);
                    message.right = -controller::get_value(controller::Axis::JS_RIGHT_Y);
                    network::serialize(message_buffer, &message);

                    network::queue_outgoing(&conn, network::MessageType::MOVEMENT, message_buffer);
		    time_passed = 1000/bandwidth_time_passed;
		    current_bandwidth = sizeof(message)*time_passed; 
		    printf("%f bandwidth in bytes per second \n",current_bandwidth);
		    printf("%f round trip time in ms \n",round_trip_time); 
                }
            }
        }

        // Update and draw GUI.
        do_gui(feeds);

        // Display our buffer.
		glfwSwapBuffers(window);

        // Send any messages that we accumulated.
        network::drain_outgoing(&conn);
    }

    // Cleanup.
	glfwTerminate();

    return 0;
}
