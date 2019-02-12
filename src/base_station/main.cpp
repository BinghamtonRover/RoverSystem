#include "controller.hpp"
#include "camera_feed.hpp"
#include "gui.hpp"

#include "../network/network.hpp"
#include "../shared.hpp"

#include <SDL.h>
#include <GL/gl.h>

#include <cstdio>
#include <cstdlib>
#include <cstdint>

#include <stack>

// Default angular resolution (vertices / radian) to use when drawing circles.
constexpr float ANGULAR_RES = 10.0f;

// We know that our base station will have this resolution.
const int WINDOW_WIDTH = 1920;
const int WINDOW_HEIGHT = 1080;

// Send movement updates 3x per second.
const int MOVMENT_SEND_INTERVAL = 1000/3;

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
    // Init just the video subsystem of SDL.
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        fprintf(stderr, "[!] Failed to init SDL: %s\n", SDL_GetError());
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
    SDL_Window* window = SDL_CreateWindow("Base Station", 0, 0, WINDOW_WIDTH, WINDOW_HEIGHT, SDL_WINDOW_OPENGL | SDL_WINDOW_FULLSCREEN);

    // Create an OpenGL context.
    SDL_GLContext gl_context = SDL_GL_CreateContext(window);
    SDL_GL_MakeCurrent(window, gl_context);

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

    bool running = true;
    while (running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            switch (event.type) {
            case SDL_QUIT:
                running = false;
                break;
            case SDL_KEYDOWN:
                if (event.key.keysym.sym == SDLK_ESCAPE) {
                    running = false;
                }
                break;
            }
        }

        // Handle incoming network messages.
        network::poll_incoming(&conn);
        
        network::Message message;
        while (network::dequeue_incoming(&conn, &message)) {
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
        }

        if (controller_loaded) {
            // Process controller input.
            controller::Event event;
            controller::Error err;
            
            // Do nothing since we just want to update current values.
            while ((err = controller::poll(&event)) == controller::Error::OK) {}

            if (err != controller::Error::DONE) {
                fprintf(stderr, "[!] Failed to read from the controller! Disabling.\n");
                controller_loaded = false;
            } else {
                if (SDL_GetTicks() - last_movement_send_time >= MOVMENT_SEND_INTERVAL) {
                    last_movement_send_time = SDL_GetTicks();

                    network::Buffer* message_buffer = network::get_outgoing_buffer();

                    network::MovementMessage message;
                    message.left = -controller::get_value(controller::Axis::JS_LEFT_Y);
                    message.right = -controller::get_value(controller::Axis::JS_RIGHT_Y);
                    network::serialize(message_buffer, &message);

                    network::queue_outgoing(&conn, network::MessageType::MOVEMENT, message_buffer);
                }
            }
        }

        // Early exit if we just decided to quit.
        if (!running) break;

        // Update and draw GUI.
        do_gui(feeds);

        // Display our buffer.
        SDL_GL_SwapWindow(window);

        // Send any messages that we accumulated.
        network::drain_outgoing(&conn);
    }

    // Cleanup.
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
