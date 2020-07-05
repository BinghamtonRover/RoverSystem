#include "../network/network.hpp"
#include "../util/util.hpp"

#include "camera_feed.hpp"
#include "controller.hpp"
#include "debug_console.hpp"
#include "../logger/logger.hpp"
#include "log_view.hpp"
#include "waypoint.hpp"
#include "waypoint_map.hpp"
//TODO: fix this design hack, currently using shared_feeds to get the rover_feed established in main for debug_console
#include "shared_feeds.hpp" 
#include "session.hpp"

#include <GL/gl.h>
#include <GLFW/glfw3.h>

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

#include <iostream>
#include <stack>
#include <string>
#include <vector>

#include <sys/types.h>
#include <unistd.h>

//Declaring base station session object
Session bs_session;

void command_callback(std::string command) {
    auto parts = gui::debug_console::split_by_spaces(command);

    if (parts.size() == 0) {
        return;
    }

    if (parts[0] == "move") {
        gui::debug_console::move(parts, bs_session.last_movement_message);
    } else if (parts[0] == "mode") {
        gui::debug_console::mode(parts, bs_session.bs_feed);
    }
}

void stderr_handler(logger::Level level, std::string message) {
    fprintf(stderr, "%s\n", message.c_str());
}

void log_view_handler(logger::Level level, std::string message) {
    float r, g, b, a = 1.0f;

    switch (level) {
        case logger::DEBUG:
            r = 0.70f;
            g = 0.70f;
            b = 0.70f;
            break;
        case logger::INFO:
            r = 1.0f;
            g = 1.0f;
            b = 1.0f;
            break;
        case logger::WARNING:
            r = 1.00f;
            g = 0.52f;
            b = 0.01f;
            break;
        case logger::ERROR:
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

    gui::log_view::print(&gui::state.global_font, LOG_VIEW_WIDTH, full_string, r, g, b, a);
}







void glfw_character_callback(GLFWwindow* window, unsigned int codepoint) {
    if (gui::state.input_state == gui::InputState::DEBUG_CONSOLE) {
        if (codepoint < 128) {
            gui::debug_console::handle_input((char) codepoint);
        }
    }
}

void send_feed(uint8_t stream_indx) {
    network::CameraControlMessage message = {
        network::CameraControlMessage::Setting::DISPLAY_STATE
    };
    message.resolution = {
        stream_indx,
        network::CameraControlMessage::sendType::SEND
    };
    network::publish(shared_feeds::bs_feed, &message);
    return;
}

void send_all_feeds() {
    for(uint8_t i = 0; i < 9; i++) {
        if(i != bs_session.primary_feed && i != bs_session.secondary_feed) {
            send_feed(i);
        }
    }
    return;
}

void dont_send_feed(uint8_t stream_indx) {
    network::CameraControlMessage message = {
        network::CameraControlMessage::Setting::DISPLAY_STATE
    };
    message.resolution = {
        stream_indx,
        network::CameraControlMessage::sendType::DONT_SEND
    };
    network::publish(shared_feeds::bs_feed, &message);
    return;
}

void dont_send_invalid() {
    for(uint8_t i = 0; i < 9; i++) {
        if(i != bs_session.primary_feed && i != bs_session.secondary_feed) {
            dont_send_feed(i);
        }
    }
    return;
}

void glfw_key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    bool z_on = glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS;
    if (gui::state.input_state == gui::InputState::DEBUG_CONSOLE) {
        if (action == GLFW_PRESS || action == GLFW_REPEAT) {
            gui::debug_console::handle_keypress(key, mods);
        }
    } else if (gui::state.input_state == gui::InputState::KEY_COMMAND) {
        // We open the menu on release to prevent the D key from being detected
        // in the character callback upon release.
        if (action == GLFW_RELEASE && key == GLFW_KEY_D) {
            gui::state.show_debug_console = true;
            gui::state.input_state = gui::InputState::DEBUG_CONSOLE;
        } else if (action == GLFW_PRESS && key == GLFW_KEY_C) {
            if (mods & GLFW_MOD_SHIFT) {
                gui::state.input_state = gui::InputState::CAMERA_MATRIX;
                send_all_feeds();
            } else {
                int temp = bs_session.primary_feed;
                bs_session.primary_feed = bs_session.secondary_feed;
                bs_session.secondary_feed = temp;
            }
        } else if (z_on && action == GLFW_RELEASE && key == GLFW_KEY_G){
            gui::waypoint_map::gridMap = !gui::waypoint_map::gridMap;
        } else if (action == GLFW_PRESS && key == GLFW_KEY_A) {
            gui::state.input_state = gui::InputState::AUTONOMY_CONTROL;
        } else if (action == GLFW_PRESS && key == GLFW_KEY_M) {
            switch (bs_session.controller_mode) {
                case ControllerMode::DRIVE:
                    bs_session.controller_mode = ControllerMode::ARM;
                    break;
                case ControllerMode::ARM:
                    bs_session.controller_mode = ControllerMode::DRIVE;
                    break;
            }
        }
    } else if (gui::state.input_state == gui::InputState::CAMERA_MATRIX) {
        if (action == GLFW_PRESS && key == GLFW_KEY_ESCAPE) {
            gui::state.input_state = gui::InputState::KEY_COMMAND;
            dont_send_invalid();
        } else if (action == GLFW_PRESS && key == GLFW_KEY_M) {
            gui::state.input_state = gui::InputState::CAMERA_MOVE;
        }
    } else if (gui::state.input_state == gui::InputState::CAMERA_MOVE) {
        if (action == GLFW_PRESS && key == GLFW_KEY_ESCAPE) {
            gui::state.input_state = gui::InputState::CAMERA_MATRIX;
        } else if (action == GLFW_PRESS) {
            int selected_feed_idx = -1;
            switch (key) {
                case GLFW_KEY_0:
                    selected_feed_idx = 0;
                    break;
                case GLFW_KEY_1:
                    selected_feed_idx = 1;
                    break;
                case GLFW_KEY_2:
                    selected_feed_idx = 2;
                    break;
                case GLFW_KEY_3:
                    selected_feed_idx = 3;
                    break;
                case GLFW_KEY_4:
                    selected_feed_idx = 4;
                    break;
                case GLFW_KEY_5:
                    selected_feed_idx = 5;
                    break;
                case GLFW_KEY_6:
                    selected_feed_idx = 6;
                    break;
                case GLFW_KEY_7:
                    selected_feed_idx = 7;
                    break;
                case GLFW_KEY_8:
                    selected_feed_idx = 8;
                    break;
                case GLFW_KEY_9:
                    selected_feed_idx = 9;
                    break;
            }

            if (selected_feed_idx >= 0 && selected_feed_idx < MAX_FEEDS) {
                bs_session.feed_to_move = selected_feed_idx;
                gui::state.input_state = gui::InputState::CAMERA_MOVE_TARGET;
            }
        }
    } else if (gui::state.input_state == gui::InputState::CAMERA_MOVE_TARGET) {
        if (action == GLFW_PRESS && key == GLFW_KEY_ESCAPE) {
            gui::state.input_state = gui::InputState::CAMERA_MATRIX;
        } else if (action == GLFW_PRESS && key == GLFW_KEY_0) {
            // If the new feed wasn't being displayed,
            // lets start displaying it
            if (bs_session.feed_to_move != bs_session.secondary_feed) {
                send_feed(bs_session.feed_to_move);
            }

            bs_session.primary_feed = bs_session.feed_to_move;
            dont_send_invalid();
            gui::state.input_state = gui::InputState::KEY_COMMAND;
        } else if (action == GLFW_PRESS && key == GLFW_KEY_1) {
            // If the new feed isn't the same as it was,
            // lets start displaying it
            if (bs_session.feed_to_move != bs_session.primary_feed) {
                send_feed(bs_session.feed_to_move);
            }

            bs_session.secondary_feed = bs_session.feed_to_move;
            dont_send_invalid();
            gui::state.input_state = gui::InputState::KEY_COMMAND;
        }
    } else if (gui::state.input_state == gui::InputState::AUTONOMY_CONTROL) {
        if (action == GLFW_PRESS && key == GLFW_KEY_ESCAPE) {
            gui::state.input_state = gui::InputState::KEY_COMMAND;
        } else if (action == GLFW_PRESS && key == GLFW_KEY_T) {
            gui::state.input_state = gui::InputState::AUTONOMY_EDIT_TARGET;
        }
    } else if (gui::state.input_state == gui::InputState::AUTONOMY_EDIT_TARGET) {
        if (action == GLFW_PRESS && key == GLFW_KEY_ESCAPE) {
            gui::state.input_state = gui::InputState::AUTONOMY_CONTROL;
        } else if (action == GLFW_PRESS && key == GLFW_KEY_TAB) {
            bs_session.autonomy_info.edit_idx = (bs_session.autonomy_info.edit_idx + 1) % 2;
        } else if (action == GLFW_PRESS && key == GLFW_KEY_ENTER) {
            // Set has_target to true, save the target, and send the command.
        } else if (action == GLFW_PRESS) {
            std::string& edit_str = bs_session.autonomy_info.edit_idx == 0 ?
                bs_session.autonomy_info.edit_lat : bs_session.autonomy_info.edit_lon;

            switch (key) {
                case GLFW_KEY_0:
                    edit_str.push_back('0');
                    break;
                case GLFW_KEY_1:
                    edit_str.push_back('1');
                    break;
                case GLFW_KEY_2:
                    edit_str.push_back('2');
                    break;
                case GLFW_KEY_3:
                    edit_str.push_back('3');
                    break;
                case GLFW_KEY_4:
                    edit_str.push_back('4');
                    break;
                case GLFW_KEY_5:
                    edit_str.push_back('5');
                    break;
                case GLFW_KEY_6:
                    edit_str.push_back('6');
                    break;
                case GLFW_KEY_7:
                    edit_str.push_back('7');
                    break;
                case GLFW_KEY_8:
                    edit_str.push_back('8');
                    break;
                case GLFW_KEY_9:
                    edit_str.push_back('9');
                    break;
                case GLFW_KEY_MINUS:
                    edit_str.push_back('-');
                    break;
                case GLFW_KEY_PERIOD:
                    edit_str.push_back('.');
                    break;
                case GLFW_KEY_BACKSPACE:
                    if (!edit_str.empty()) edit_str.pop_back();
                    break;
            }
        }
    }
}


int main() {

    util::Clock::init(&bs_session.global_clock);

    logger::register_handler(stderr_handler);

    gui::debug_console::set_callback(command_callback);

    // Load config.
    bs_session.config = bs_session.load_config("res/bs.sconfig");

    // Clear the stopwatch.
    bs_session.stopwatch.state = StopwatchState::STOPPED;
    bs_session.stopwatch.start_time = 0;
    bs_session.stopwatch.pause_time = 0;

    // Init GLFW.
    if (!glfwInit()) {
        logger::log(logger::ERROR, "Failed to init GLFW!");
        return 1;
    }

    GLFWmonitor* monitor_to_use = glfwGetPrimaryMonitor();

    if (bs_session.config.preferred_monitor[0] != 0) {
        // A monitor was specified. Does it exist?
        int num_monitors;
        GLFWmonitor** monitors = glfwGetMonitors(&num_monitors);

        bool found = false;

        for (int i = 0; i < num_monitors; i++) {
            if (strncmp(bs_session.config.preferred_monitor, glfwGetMonitorName(monitors[i]), Config::MAX_PREFERRED_MONITOR_LEN) == 0) {
                monitor_to_use = monitors[i];
                found = true;
                break;
            }
        }

        if (!found) logger::log(logger::WARNING, "Preferred monitor %s not found!", bs_session.config.preferred_monitor);
    }

    // Get the correct resolution for the monitor we want to use.
    const GLFWvidmode* video_mode = glfwGetVideoMode(monitor_to_use);

    // Create a fullscreen window. Title isn't displayed, so doesn't really
    // matter.
    GLFWwindow* window =
        glfwCreateWindow(video_mode->width, video_mode->height, "Base Station", monitor_to_use, NULL);

    // Update the window so everyone can access it.
    gui::state.window = window;

    // Set sticky keys mode. It makes our input work as intended.
    // glfwSetInputMode(window, GLFW_STICKY_KEYS, 1);

    // Disable mouse.
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_HIDDEN);

    glfwSetCharCallback(window, glfw_character_callback);
    glfwSetKeyCallback(window, glfw_key_callback);

    // Create an OpenGL context.
    glfwMakeContextCurrent(window);

    // OpenGL Setup.
    glViewport(0, 0, video_mode->width, video_mode->height);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, gui::WINDOW_WIDTH, gui::WINDOW_HEIGHT, 0, 0, 0.5);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Initial setup for GUI here so that network errors are printed to log view.
    bs_session.map_texture_id = gui::load_texture("res/binghamton.jpg");
    bs_session.stopwatch_texture_id = gui::load_texture_alpha("res/stopwatch_white.png");

    // Load this down here so that sizing is correct.
    bool loaded_font = gui::load_font(&gui::state.global_font, "res/FiraMono-Regular.ttf", 100);
    if (!loaded_font) {
        logger::log(logger::ERROR, "Failed to load font!");
        return 1;
    }

    logger::register_handler(log_view_handler);

    // Initialize camera stuff.
    camera_feed::init();

    // Create the camera streams.
    for (int i = 0; i < MAX_FEEDS; i++) {
        static char init_feed_name_buffer[camera_feed::FEED_NAME_MAX_LEN + 1];
        snprintf(init_feed_name_buffer, sizeof(init_feed_name_buffer), "UNKNOWN-%d", i);

        camera_feed::init_feed(bs_session.camera_feeds + i, init_feed_name_buffer, 1280, 720);
    }

    // Initialize network functionality.
    {
        auto err = network::init(
            &bs_session.bs_feed,
            network::FeedType::OUT,
            bs_session.config.interface,
            bs_session.config.base_station_multicast_group,
            bs_session.config.base_station_port,
            &bs_session.global_clock);

        if (err != network::Error::OK) {
            logger::log(logger::ERROR, "Failed to init base station feed: %s", network::get_error_string(err));
            return 1;
        }
        shared_feeds::bs_feed = &bs_session.bs_feed;
        logger::log(
            logger::INFO,
            "Network: publishing base station feed on %s:%d",
            bs_session.config.base_station_multicast_group,
            bs_session.config.base_station_port);
    }

    {
        auto err = network::init(
            &bs_session.r_feed,
            network::FeedType::IN,
            bs_session.config.interface,
            bs_session.config.rover_multicast_group,
            bs_session.config.rover_port,
            &bs_session.global_clock);

        if (err != network::Error::OK) {
            logger::log(logger::ERROR, "Failed to init rover feed: %s", network::get_error_string(err));
            return 1;
        }

        logger::log(
            logger::INFO,
            "Network: subscribed to rover feed on %s:%d",
            bs_session.config.rover_multicast_group,
            bs_session.config.rover_port);

        
    }
    
    // Keep track of when we last sent movement info.
    util::Timer movement_send_timer;
    util::Timer::init(&movement_send_timer, MOVEMENT_SEND_INTERVAL, &bs_session.global_clock);

    util::Timer arm_send_timer;
    util::Timer::init(&arm_send_timer, ARM_SEND_INTERVAL, &bs_session.global_clock);

    // Grab network stats every second.
    util::Timer network_stats_timer;
    util::Timer::init(&network_stats_timer, NETWORK_STATS_INTERVAL, &bs_session.global_clock);

    // Init the controller.
    // TODO: QUERY /sys/class/input/js1/device/id/{vendor,product} TO FIND THE RIGHT CONTROLLER.
    bool controller_loaded = false;
    if (controller::init("/dev/input/js0") == controller::Error::OK) {
        controller_loaded = true;
        logger::log(logger::INFO, "Controller connected.");
    } else {
        logger::log(logger::WARNING, "No controller connected!");
    }

    gui::debug_console::log("Debug log initialized.", 0, 1.0, 0);

    bool help_menu_up = false;
    bool stopwatch_menu_up = false;

    // Add the help menu commands here
    std::vector<const char*> commands;
    std::vector<const char*> debug_commands;
    commands.push_back("d: Show debug console");
    commands.push_back("ctrl + q: Exit");
    commands.push_back("<shift>c: Open camera matrix");
    commands.push_back("c: Swap camera feeds");
    commands.push_back("s: Open stopwatch menu");
    commands.push_back("z + UP ARROW: Zoom in map");
    commands.push_back("z + DOWN ARROW: Zoom out map");
    commands.push_back("z + r: Reset map");
    commands.push_back("z + g: Toggle map display");
    debug_commands.push_back("'test': displays red text");
    debug_commands.push_back("'aw <number> <number>': adds a waypoint (in latitude and longitude)");
    debug_commands.push_back("'gs_on': Changes camera feeds to greyscale");
    debug_commands.push_back("'gs_off': Changes camera feeds to RGB");
    debug_commands.push_back("'jpeg_quality <0-100>: Changes the quality of the cameras");

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        bool z_on = glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS;

        if (z_on && (glfwGetKey(window,GLFW_KEY_UP) == GLFW_PRESS)) {
            gui::waypoint_map::zoom_in();    
        } else if (z_on && (glfwGetKey(window,GLFW_KEY_DOWN) == GLFW_PRESS)) {
            gui::waypoint_map::zoom_out();
        } else if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS && gui::state.input_state == gui::InputState::KEY_COMMAND) {
            if (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS) {
                gui::log_view::moveTop();
            } else {
                gui::log_view::moveUpOne();
            }
        } else if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS && gui::state.input_state == gui::InputState::KEY_COMMAND) {
            if (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS) {
                gui::log_view::moveBottom();
            } else {
                gui::log_view::moveDownOne();
            }
        } else if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) {
            if (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS) {
                break;
            }
        } else if (glfwGetKey(window, GLFW_KEY_H) == GLFW_PRESS) {
            if (gui::state.input_state == gui::InputState::KEY_COMMAND) help_menu_up = true;
        } else if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
            if (help_menu_up) help_menu_up = false;
            if (stopwatch_menu_up) {
                stopwatch_menu_up = false;
                gui::state.input_state = gui::InputState::KEY_COMMAND;
            }
        } else if (
            glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS && gui::state.input_state == gui::InputState::KEY_COMMAND) {
            stopwatch_menu_up = true;
            gui::state.input_state = gui::InputState::STOPWATCH_MENU;
        }

        if (gui::state.input_state == gui::InputState::STOPWATCH_MENU) {
            if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) {
                switch (bs_session.stopwatch.state) {
                    case StopwatchState::RUNNING:
                        bs_session.stopwatch.state = StopwatchState::PAUSED;
                        bs_session.stopwatch.pause_time = bs_session.global_clock.get_millis();
                        break;
                    case StopwatchState::PAUSED:
                        bs_session.stopwatch.state = StopwatchState::RUNNING;
                        bs_session.stopwatch.start_time =
                            bs_session.global_clock.get_millis() - (bs_session.stopwatch.pause_time - bs_session.stopwatch.start_time);
                        break;
                    case StopwatchState::STOPPED:
                        bs_session.stopwatch.state = StopwatchState::RUNNING;
                        bs_session.stopwatch.start_time = bs_session.global_clock.get_millis();
                        break;
                }

                stopwatch_menu_up = false;
                gui::state.input_state = gui::InputState::KEY_COMMAND;
            } else if (glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS) {
                switch (bs_session.stopwatch.state) {
                    case StopwatchState::RUNNING:
                        bs_session.stopwatch.start_time = bs_session.global_clock.get_millis();
                        break;
                    case StopwatchState::PAUSED:
                        bs_session.stopwatch.state = StopwatchState::STOPPED;
                        break;
                    case StopwatchState::STOPPED:
                        break;
                }

                stopwatch_menu_up = false;
                gui::state.input_state = gui::InputState::KEY_COMMAND;
            }
        }

        // Handle incoming network messages from the rover feed.
        network::IncomingMessage message;
        while (true) {
            auto neterr = network::receive(&bs_session.r_feed, &message);
            if (neterr != network::Error::OK) {
                if (neterr == network::Error::NOMORE) {
                    break;
                } else {
                    logger::log(
                        logger::WARNING,
                        "Failed to read network packets: %s",
                        network::get_error_string(neterr));
                    break;
                }
            }

            switch (message.type) {
                case network::MessageType::CAMERA: {
                    // Static buffer so we don't have to allocate and reallocate every
                    // frame.
                    static uint8_t camera_message_buffer[camera_feed::CAMERA_MESSAGE_FRAME_DATA_MAX_SIZE];

                    network::CameraMessage camera_message;
                    camera_message.data = camera_message_buffer;
                    network::deserialize(&message.buffer, &camera_message);

                    auto camerr = camera_feed::handle_section(
                        bs_session.camera_feeds + camera_message.stream_index,
                        camera_message.data,
                        camera_message.size,
                        camera_message.section_index,
                        camera_message.section_count,
                        camera_message.frame_index);

                    if (camerr != camera_feed::Error::OK) {
                        logger::log(logger::WARNING, "Failed to handle video frame section!");
                    }

                    break;
                }
                case network::MessageType::LIDAR: {
                    bs_session.lidar_points.clear();

                    network::LidarMessage lidar_message;
                    network::deserialize(&message.buffer, &lidar_message);

                    for (int i = 0; i < network::LidarMessage::NUM_LIDAR_POINTS; i++) {
                        bs_session.lidar_points.push_back(lidar_message.points[i]);
                    }

                    break;
                }
                case network::MessageType::TICK: {
                    network::TickMessage tick_message;
                    network::deserialize(&message.buffer, &tick_message);

                    bs_session.last_rover_tick = tick_message.ticks_per_second;
                    break;
                }
                case network::MessageType::LOCATION: {
                    network::LocationMessage location_message;
                    network::deserialize(&message.buffer,&location_message);
                    waypoint::set_rover_coordinates(location_message.latitude,location_message.longitude);
                    waypoint::rover_fix = location_message.fix_status;
                    //waypoint::rover_latitude = location_message.latitude;
                    //rover_longitude = location_message.longitude;

                    break;
                }
                case network::MessageType::MODE: {
                    network::ModeMessage mode_message;
                    network::deserialize(&message.buffer, &mode_message);
                    bs_session.mode = mode_message.mode;
                    break;
                }
                default:
                    break;
            }
        }

        // Update network stuff.
        network::update_status(&bs_session.r_feed);
        network::update_status(&bs_session.bs_feed);

        if (network_stats_timer.ready()) {
            bs_session.r_tp = (float) bs_session.r_feed.bytes_transferred / ((float) NETWORK_STATS_INTERVAL * 1000.0f);
            bs_session.bs_tp = (float) bs_session.bs_feed.bytes_transferred / ((float) NETWORK_STATS_INTERVAL * 1000.0f);
            bs_session.t_tp = bs_session.r_tp + bs_session.bs_tp;

            bs_session.r_feed.bytes_transferred = 0;
            bs_session.bs_feed.bytes_transferred = 0;
        }

        if (controller_loaded) {
            // Process controller input.
            controller::Event event;
            controller::Error err;

            while ((err = controller::poll(&event)) == controller::Error::OK) {
                if (event.type == controller::EventType::BUTTON) {
                    if (event.button == controller::Button::BACK && event.value != 0) {
                        int temp = bs_session.primary_feed;
                        bs_session.primary_feed = bs_session.secondary_feed;
                        bs_session.secondary_feed = temp;
                    } else if (event.button == controller::Button::XBOX && event.value != 0) {
                        switch (bs_session.controller_mode) {
                            case ControllerMode::DRIVE:
                                bs_session.controller_mode = ControllerMode::ARM;
                                break;
                            case ControllerMode::ARM:
                                bs_session.controller_mode = ControllerMode::DRIVE;
                                break;
                        }
                    }
                }

                switch (bs_session.controller_mode) {
                    case ControllerMode::DRIVE:
                        controller::handle_drive_controller_event(event, &bs_session);
                        break;
                    case ControllerMode::ARM:
                        controller::handle_arm_controller_event(event, &bs_session);
                        break;
                }
            }

            if (err != controller::Error::DONE) {
                logger::log(logger::ERROR, "Failed to read from the controller! Disabling");
                controller_loaded = false;
            }
        }

        if (movement_send_timer.ready()) {
            // printf("sending movement with %d, %d\n", last_movement_message.left, last_movement_message.right);

            network::publish(&bs_session.bs_feed, &bs_session.last_movement_message);
        }

        if (arm_send_timer.ready()) {
            network::publish(&bs_session.bs_feed, &bs_session.last_arm_message);
        }

        // Update and draw GUI.
        gui::do_gui(&bs_session);

        if (help_menu_up) gui::do_help_menu(commands, debug_commands, &bs_session);

        if (stopwatch_menu_up) {
            gui::do_stopwatch_menu(&bs_session);
        }

        glColor4f(0.0f, 0.0f, 0.0f, 1.0f);

        // Display our buffer.
        glfwSwapBuffers(window);
    }

    // Cleanup.
    //var.fn();
    glfwTerminate();

    return 0;
}
