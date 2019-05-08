#include "../network/network.hpp"
#include "../shared.hpp"

#include "camera_feed.hpp"
#include "controller.hpp"
#include "log_view.hpp"
#include "debug_console.hpp"
#include "gui.hpp"
#include "log.hpp"
#include "math.hpp"

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
#include <time.h>

// Default angular resolution (vertices / radian) to use when drawing circles.
constexpr float ANGULAR_RES = 10.0f;

// We know that our base station will have this resolution.
const int WINDOW_WIDTH = 1920;
const int WINDOW_HEIGHT = 1080;

// For control smoothing.
const float ALPHA = 30;

// Speed for the DPAD up/down.
const int16_t JOINT_DRIVE_SPEED = 50;

// Send movement updates 3x per second.
const int MOVEMENT_SEND_INTERVAL = 1000 / 20;
// Heartbeat interval
const int HEARTBEAT_SEND_INTERVAL = 1000 / 3;
const int RECONNECT_INTERVAL = 1000 / 3;
// Amount of time between heartbeats until disconnection flag is set
const int DISCONNECT_TIMER = 5000;

const int LOG_VIEW_WIDTH = 572;
const int LOG_VIEW_HEIGHT = 458;

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

	time_t current_time;
	time(&current_time);
	struct tm* time_info = localtime(&current_time);

	char time_string_buffer[200];
	strftime(time_string_buffer, sizeof(time_string_buffer), "[%H:%M:%S] ", time_info);

	std::string full_string = std::string(time_string_buffer) + message;

	gui::log_view::print(full_string, r, g, b, a);
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

void do_info_panel(gui::Layout* layout, gui::Font* font) {
	int x = layout->current_x;
	int y = layout->current_y;

	int w = 445;
	int h = 300;

	gui::do_solid_rect(layout, w, h, 68.0f / 255.0f, 68.0f / 255.0f, 68.0f / 255.0f);

	char bandwidth_buffer[50];
	sprintf(bandwidth_buffer, "Network bandwidth: %.3fM/s", conn.last_bandwidth);

	glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
	gui::draw_text(font, bandwidth_buffer, x + 5, y + 5, 20);


	time_t current_time;
	time(&current_time);
	struct tm* time_info = localtime(&current_time);

	char time_string_buffer[200];
	strftime(time_string_buffer, sizeof(time_string_buffer), "%I:%M:%S", time_info);

	int tw = gui::text_width(font, time_string_buffer, 20);

	gui::draw_text(font, time_string_buffer, x + 5, y + h - 20 - 5, 20);
}

std::vector<uint16_t> lidar_points;

network::LocationMessage location{};

void do_lidar(gui::Layout* layout) {
	int wx = layout->current_x;
	int wy = layout->current_y;

    gui::do_solid_rect(layout, 300, 300, 0, 0, 0);

	glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
	glLineWidth(2.0f);

	for (int q = -3; q <= 3; q++) {
		for (int r = -3; r <= 3; r++) {
			float x = 1.2f * (3.0f / 2.0f) * q;
			float y = 1.2f * ((math::sqrtf(3.0f)/2.0f) * q + math::sqrtf(3) * r);

			float ppm = 150.0f / 10.0f;

			float px = ppm * x;
			float py = ppm * y;

			glBegin(GL_LINE_LOOP);

			for (int i = 0; i < 6; i++) {
				float angle = math::PI * (float)i / 3.0f;

				float vx = wx + 150.0f + px + ppm * 1.2 * math::cosf(angle);
				float vy = wy + 150.0f + py + ppm * 1.2 * math::sinf(angle);

//				glVertex2f(vx, vy);
			}

			glEnd();
		}
	}

	glColor4f(0.0f, 0.0f, 1.0f, 1.0f);

	glBegin(GL_QUADS);

	for (size_t i = 0; i < lidar_points.size(); i++) {
		float angle = 225.0f - (float)i;
		float theta = angle * math::PI / 180.0f;

		theta -= math::PI;

		uint16_t dist = lidar_points[i];

		float r = dist * 150.0f / 10000.0f;

		float hs = 1.0f;

		float x = wx + 150.0f + r * math::cosf(theta);
		float y = wy + 150.0f + r * math::sinf(theta);

		if (dist < 100) continue;

		glVertex2f(x - hs, y - hs);
		glVertex2f(x + hs, y - hs);
		glVertex2f(x + hs, y + hs);
		glVertex2f(x - hs, y + hs);
	}

	float hs = 1.2 * 15.0f / 2.0f;

	float x = wx + 150.0f;
	float y = wy + 150.0f;

	glColor4f(1.0f, 0.0f, 0.0f, 1.0f);

	glVertex2f(x - hs, y - hs);
	glVertex2f(x + hs, y - hs);
	glVertex2f(x + hs, y + hs);
	glVertex2f(x - hs, y + hs);

	glEnd();
}

int primary_feed = 0;
int secondary_feed = 1;

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
	gui::log_view::do_log(&layout, LOG_VIEW_WIDTH, LOG_VIEW_HEIGHT, font);

    layout.reset_y();
    layout.advance_x(10);
    layout.push();

    // Draw the main camera feed.
    gui::do_textured_rect(&layout, 1298, 730, feed[primary_feed].gl_texture_id);

    layout.reset_x();
    layout.advance_y(10);
    layout.push();

    // Draw the other camera feed.
    layout.reset_y();
    gui::do_textured_rect(&layout, 533, 300, feed[secondary_feed].gl_texture_id);

    layout.reset_y();
    layout.advance_x(10);

	// Draw the lidar.
	do_lidar(&layout);

	layout.reset_y();
	layout.advance_x(10);

	// Draw the info panel.
	do_info_panel(&layout, font);

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

	if (gui::state.input_state == gui::InputState::KEY_COMMAND) {
		if (action == GLFW_PRESS && key == GLFW_KEY_C) {
			int temp = primary_feed;
			primary_feed = secondary_feed;
			secondary_feed = temp;
		}
	}
}

// Takes values between 0 and 255 and returns them between 0 and 255.
float smooth_rover_input(float value) {
	// We want to exponentially smooth this.
	// We do that by picking alpha in (1, whatever).
	// The higher the alpha, the more exponential the thing.
	// We then make sure that f(0) = 0 and f(1) = 255.

	return (255.0f / (ALPHA - 1)) * (math::powf(1 / ALPHA, -value/255.0f) - 1);
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

	map_texture_id = gui::load_texture("res/binghamton.jpg");

    gui::Font font;
    bool loaded_font = gui::load_font(&font, "res/FiraMono-Regular.ttf", 100);
    if (!loaded_font) {
		log::log(log::ERROR, "Failed to load font!");
        return 1;
    }

	gui::log_view::calc_sizing(&font, LOG_VIEW_WIDTH, LOG_VIEW_HEIGHT);

    // Init the controller.
	// TODO: QUERY /sys/class/input/js1/device/id/{vendor,product} TO FIND THE RIGHT CONTROLLER.
    bool controller_loaded = false;
    if (controller::init("/dev/input/js1") == controller::Error::OK) {
        controller_loaded = true;
		log::log(log::INFO, "Controller connected.");
    } else {
		log::log(log::WARNING, "No controller connected!");
    }

    gui::debug_console::log("Debug log initialized.", 0, 1.0, 0);

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
            if (gui::state.input_state == gui::InputState::KEY_COMMAND) {
                gui::state.show_debug_console = true;
                gui::state.input_state = gui::InputState::DEBUG_CONSOLE;
            }
        } else if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) {
			if (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS) {
				break;	
			}
		}

		network::update_bandwidth(&conn, get_ticks());

        // Handle incoming network messages.
        network::Message message;
        while (true) {
			network::Error neterr = network::poll(&conn, &message);
			if (neterr != network::Error::OK) {
				if (neterr == network::Error::NOMORE) {
					break;
				} else {
					log::log(log::WARNING, "Failed to read network packets! %d", neterr);
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
				case network::MessageType::LIDAR: {
									  lidar_points.clear();

									  network::LidarMessage lidar_message;
									  network::deserialize(message.buffer, &lidar_message);

									  for (int i = 0; i < network::NUM_LIDAR_POINTS; i++) {
										  lidar_points.push_back(lidar_message.points[i]);
									  }
									  break;
								  }
				case network::MessageType::LOCATION: {
									     network::LocationMessage location_message;
									     network::deserialize(message.buffer, &location_message);

									     printf("%f, %f, %f, %f, %f, %f\n", location_message.x, location_message.y, location_message.z, location_message.pitch, location_message.yaw, location_message.roll);

									     break;
								     }
				case network::MessageType::SENSOR: {
									   network::SensorMessage sensor_message;
									   network::deserialize(message.buffer, &sensor_message);
									   printf("%d %f %f %f\n",sensor_message.moisture, sensor_message.pressure, sensor_message.altitude, sensor_message.temperature); 
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

					if (event.axis == controller::Axis::JS_LEFT_Y) {
						if (
							controller::get_value(controller::Axis::DPAD_X) != 0
							|| controller::get_value(controller::Axis::DPAD_Y) != 0
						) {
							continue;
						}

						int16_t smoothed = (int16_t) smooth_rover_input((float) (abs_val >> 7));
						log::log(log::DEBUG, "Left orig: %d, smooth: %d", abs_val, smoothed);

						last_movement_message.left = smoothed;
						if (!forward) last_movement_message.left *= -1;
					} else if (event.axis == controller::Axis::JS_RIGHT_Y) {
						if (
							controller::get_value(controller::Axis::DPAD_X) != 0
							|| controller::get_value(controller::Axis::DPAD_Y) != 0
						) {
							continue;
						}

						int16_t smoothed = (int16_t) smooth_rover_input((float) (abs_val >> 7));
						log::log(log::DEBUG, "Right orig: %d, smooth: %d", abs_val, smoothed);

						last_movement_message.right = smoothed;
						if (!forward) last_movement_message.right *= -1;
					} else if (event.axis == controller::Axis::DPAD_Y) {
						int16_t val = -event.value;

						if (val > 0) {
							last_movement_message.left = JOINT_DRIVE_SPEED;
							last_movement_message.right = JOINT_DRIVE_SPEED;
						} else if (val < 0) {
							last_movement_message.left = -JOINT_DRIVE_SPEED;
							last_movement_message.right = -JOINT_DRIVE_SPEED;
						} else {
							last_movement_message.left = 0;
							last_movement_message.right = 0;
						}
					} else if (event.axis == controller::Axis::DPAD_X) {
						if (event.value > 0) {
							last_movement_message.left = JOINT_DRIVE_SPEED;
							last_movement_message.right = -JOINT_DRIVE_SPEED;
						} else if (event.value < 0) {
							last_movement_message.left = -JOINT_DRIVE_SPEED;
							last_movement_message.right = JOINT_DRIVE_SPEED;
						} else {
							last_movement_message.left = 0;
							last_movement_message.right = 0;
						}
					}

				} else if (event.type == controller::EventType::BUTTON) {
					if (event.button == controller::Button::BACK && event.value != 0) {
						int temp = primary_feed;
						primary_feed = secondary_feed;
						secondary_feed = temp;
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

			// printf("sending movement with %d, %d\n", last_movement_message.left, last_movement_message.right);

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
