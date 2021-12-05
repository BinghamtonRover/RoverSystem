#include "../network/network.hpp"
#include "../util/util.hpp"

#include "debug_console.hpp"
#include "log_view.hpp"
#include "waypoint.hpp"
#include "waypoint_map.hpp"
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


int main() {
    //Declaring base station session object
    Session bs_session;

    util::Clock::init(&bs_session.global_clock);

    logger::register_handler(logger::stderr_handler);

    gui::debug_console::set_callback(gui::debug_console::command_callback);

    // Load config.
    bs_session.load_config("res/bs.sconfig");

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


    glfwSetCharCallback(window, gui::glfw_character_callback);
    glfwSetWindowUserPointer(window, &bs_session);
    glfwSetKeyCallback(window, gui::glfw_key_callback);

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

    logger::register_handler(gui::log_view::log_view_handler);

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
    {
        auto err = network::init(
            &bs_session.v_feed,
            network::FeedType::IN,
            bs_session.config.interface,
            bs_session.config.video_multicast_group,
            bs_session.config.video_port,
            &bs_session.global_clock);

        if (err != network::Error::OK) {
            logger::log(logger::ERROR, "Failed to init rover feed: %s", network::get_error_string(err));
            return 1;
        }
        logger::log(
            logger::INFO,
            "Network: subscribed to video computer feed on %s:%d",
            bs_session.config.video_multicast_group,
            bs_session.config.video_port);

        
    }
    bs_session.controller_mgr.init();

    // Apply the default driving configuration to the controller
    for (auto &c : bs_session.controller_mgr.devices) {
        if (c.present() && c.count_axes() == 6) {
            // The joysticks typically require a deadzone to be accurate, but the triggers do not
            c[0].recenter(0.0F, 0.05F);
            c[0].set_action(bs_session.steering_action);
            c[2].set_action(bs_session.reverse_action);
            c[5].set_action(bs_session.forward_action);
        }
    }

    // Init last movement info timers.
    util::Timer::init(&bs_session.movement_send_timer, MOVEMENT_SEND_INTERVAL, &bs_session.global_clock);
    util::Timer::init(&bs_session.arm_send_timer, ARM_SEND_INTERVAL, &bs_session.global_clock);
    util::Timer::init(&bs_session.network_stats_timer, NETWORK_STATS_INTERVAL, &bs_session.global_clock);

    gui::debug_console::log("Debug log initialized.", 0, 1.0, 0);

    bool help_menu_up = false;
    bool stopwatch_menu_up = false;

    bool image_display_up = false;
    char * firstImageName = NULL;
    unsigned int firstImageID;
    unsigned int firstImageTime;
    char * secondImageName = NULL;
    unsigned int secondImageID;

    // Add the help menu commands here
    std::vector<const char*> commands;
    std::vector<const char*> debug_commands;
    commands.push_back("d: Show debug console");
    commands.push_back("ctrl + q: Exit");
    commands.push_back("<shift>c: Open camera matrix");
    commands.push_back("c: Swap camera feeds");
    commands.push_back("s: Open stopwatch menu");
    commands.push_back("p: Takes initial picture from rover feed");
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
        bool quit = false;
        if (gui::state.input_state == gui::InputState::KEY_COMMAND) {
            gui::poll_keys_callback(bs_session, window, help_menu_up, stopwatch_menu_up, image_display_up, firstImageTime, firstImageName, secondImageName, quit);
        }
        if (quit) { break; }

        bs_session.controller_mgr.update_controls();

        //Checks if it has been 10 seconds since a picture was taken
        if(firstImageName != NULL && secondImageName == NULL && bs_session.global_clock.get_millis() >= (firstImageTime + (10 * 1000))){
            secondImageName = gui::gen_bitmap_from_camera_feed(&bs_session);
            std::string msg_beginning = "Saved second image as: ";
            char * pic_msg = (char *) malloc(1 + std::strlen(msg_beginning.c_str()) + std::strlen(secondImageName));
            std::strcpy(pic_msg, msg_beginning.c_str());
            std::strcat(pic_msg, secondImageName);
            logger::log(logger::INFO, pic_msg);
            std::free(pic_msg);
            firstImageID = gui::load_texture(firstImageName);
            secondImageID = gui::load_texture(secondImageName);
            image_display_up = true;
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
        {
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
                        bs_session.last_subsystem_tick = tick_message.ticks_per_second;
                        break;
                    }
                    case network::MessageType::LOCATION: {
                        network::LocationMessage location_message;
                        network::deserialize(&message.buffer,&location_message);
                        waypoint::set_rover_coordinates(location_message.latitude,location_message.longitude);
                        waypoint::rover_fix = location_message.fix_status;
                        break;
                    }
                    case network::MessageType::FOCUS_MODE: {
                        network::FocusModeMessage focus_mode_message;
                        network::deserialize(&message.buffer, &focus_mode_message);
                        bs_session.subsystem_focus_mode = focus_mode_message.focus_mode;
                        break;
                    }
                    default:
                        break;
                }
            }
        }
        // Handle incoming network messages from the video computer feed.
        {
            network::IncomingMessage message;
            while (true) {
                auto neterr = network::receive(&bs_session.v_feed, &message);
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
                    case network::MessageType::TICK: {
                        network::TickMessage tick_message;
                        network::deserialize(&message.buffer, &tick_message);
                        bs_session.last_video_tick = tick_message.ticks_per_second;
                        break;
                    }
                    case network::MessageType::FOCUS_MODE: {
                        network::FocusModeMessage focus_mode_message;
                        network::deserialize(&message.buffer, &focus_mode_message);
                        bs_session.video_focus_mode = focus_mode_message.focus_mode;
                        break;
                    }
                    default:
                        break;
                }
            }
        }

        // Update network stuff.
        network::update_status(&bs_session.r_feed);
        network::update_status(&bs_session.bs_feed);
        network::update_status(&bs_session.v_feed);

        if (bs_session.network_stats_timer.ready()) {
            bs_session.r_tp = (float) bs_session.r_feed.bytes_transferred / ((float) NETWORK_STATS_INTERVAL * 1000.0f);
            bs_session.bs_tp = (float) bs_session.bs_feed.bytes_transferred / ((float) NETWORK_STATS_INTERVAL * 1000.0f);
            bs_session.v_tp = (float) bs_session.v_feed.bytes_transferred / ((float) NETWORK_STATS_INTERVAL * 1000.0f);
            bs_session.t_tp = bs_session.r_tp + bs_session.bs_tp + bs_session.v_tp;

            bs_session.r_feed.bytes_transferred = 0;
            bs_session.bs_feed.bytes_transferred = 0;
            bs_session.v_feed.bytes_transferred = 0;
        }

        if (bs_session.movement_send_timer.ready()) {
            bs_session.calc_drive_speed();
            bs_session.drive_sub_info["Target Right Speed"] = bs_session.last_movement_message.right;
            bs_session.drive_sub_info["Target Left Speed"] = bs_session.last_movement_message.left;
            network::publish(&bs_session.bs_feed, &bs_session.last_movement_message);
        }

        // Update and draw GUI.
        gui::do_gui(&bs_session);

        if (bs_session.log_file.is_open() && bs_session.log_interval_timer.ready()) {
            bs_session.export_data();
        }

        if (help_menu_up) gui::do_help_menu(commands, debug_commands, &bs_session);

        if (stopwatch_menu_up) {
            gui::do_stopwatch_menu(&bs_session);
        }

        if (image_display_up && secondImageName != NULL) gui::do_image_display(firstImageID, secondImageID, &bs_session);

        glColor4f(0.0f, 0.0f, 0.0f, 1.0f);

        // Display our buffer.
        glfwSwapBuffers(window);
    }

    // Cleanup.
    glfwTerminate();

    return 0;
}