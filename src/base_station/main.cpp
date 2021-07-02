#include "../network/network.hpp"
#include "../util/util.hpp"

#include "controller.hpp"
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

/*******main.cpp************************************************************************
1. The main function of the rover base station
    1.a Declaring base station session object
    1.b Load config
    1.c Clear the stopwatch
    1.d Init GLFW
    1.e A monitor was specified. Does it exist?
        1.e.i Finds the correct monitor to used based on name and size
        1.e.ii If no monitor is found, we log a warning
    1.f Get the correct resolution for the monitor we want to use.
    1.g Create a fullscreen window. Title isn't displayed, so doesn't really matter.
    1.h Update the window so everyone can access it.
    1.i Disable mouse.
    1.j Create an OpenGL context.
    1.k OpenGL Setup.
    1.l Initial setup for GUI here so that network errors are printed to log view.
    1.m Load this down here so that sizing is correct.
    1.n If we cannot find the loaded font, log an error message
    1.o Initialize camera stuff.
    1.p Create the camera streams.
    1.q Initialize network functionality.
        1.q.i If we cannot init the base station feed, we log an error
        1.q.ii If we cannot init the rover feed, log an error
        1.q.iii If we cannot init the video feed, we log an error
    1.r Init last movement info timers.
    1.s Init the controller.
    1.t Add the help menu commands here
    1.u Main loop for the basestation
        1.u.i If z and the up arrow are pressed, zoom in on the waypoint map
        1.u.ii If z and the down arrow are pressed, zoom in on the waypoint map
        1.u.iii If the up arrow is pressed and the input state is key command
            1.u.iii.1 If shift is pressed, move to the top of the log, else move up one line
        1.u.iv If the down arrow is pressed and the input state is key command
            1.u.iv.1 If shift is pressed, move to the bottom of the log, else move down one line
        1.u.v If the q key is pressed
            1.u.v.1 If left control is also pressed, exit the program
        1.u.vi If the h key is pressed
            1.u.vi.1 If the input state is key command, bring up the help menu
        1.u.vii If the escape key is pressed
            1.u.vii.1 If the help menu is up, close it
            1.u.vii.2 If the stop watch menu is up, close it and set the input state to key command
        1.u.viii If the s key is pressed and the input state is key command, show the stopwatch menu
        1.u.ix If the input state is stopwatch menu
            1.u.ix.1 If the space key is pressed, toggle the stopwatch state based on current state
            1.u.ix.2 If the r key is pressed, reset the time
        1.u.x Handle incoming network messages from the rover feed.
            1.u.x.1 Parses a full message from the subsystem computer
                1.u.x.1.a Switches how messages are handled based on the message type
                    1.u.x.1.a.i Adds lidar points from the message to the base station
        1.u.xi Handle incoming network messages from the video computer feed.
            1.u.xi.1 Parses a full message from the video computer
                1.u.xi.1.a If the video computer returned an error
                    1.u.xi.1.a.i If there are no more messages to parse, break the loop
                    1.u.xi.1.a.ii If a different error occured, log the error
                1.u.xi.1.b Swtiches the handling of messages based on the type
                    1.u.xi.1.b.i If there is a camrea feed error, log it
        1.u.xii Update network stuff
        1.u.xiii If we network stats are ready, we can set them in the basestation
        1.u.xiv Process controller input
            1.u.xiv.1 While the controller has no errors
                1.u.xiv.1.a If a button is pressed
                    1.u.xiv.1.a.i If the back button is pressed, log a debug message and swap the feeds
                    1.u.xiv.1.a.ii If the xbox button is pressed, swap controller modes
                1.u.xiv.1.b Swaps the way events are handled based on the controller mode
            1.u.xiv.2 If we cannot read from the controller, disconnect
        1.u.xv Allows for controller reconnects
        1.u.xvi Publishes a movement message to the network
        1.u.xvii Update and draw GUI.
        1.u.xviii Exports data to the log file
        1.u.xix If the help menu is enabled, draw it
        1.u.xx If the stopwatch is enabled, draw it
        1.u.xxi Display our buffer.
    1.v Cleanup.
***************************************************************************************/

// 1. The main function of the rover base station
int main() {
    // 1.a Declaring base station session object
    Session bs_session;

    util::Clock::init(&bs_session.global_clock);

    logger::register_handler(logger::stderr_handler);

    gui::debug_console::set_callback(gui::debug_console::command_callback);

    // 1.b Load config.
    bs_session.load_config("res/bs.sconfig");

    // 1.c Clear the stopwatch.
    bs_session.stopwatch.state = StopwatchState::STOPPED;
    bs_session.stopwatch.start_time = 0;
    bs_session.stopwatch.pause_time = 0;

    // 1.d Init GLFW.
    if (!glfwInit()) {
        logger::log(logger::ERROR, "Failed to init GLFW!");
        return 1;
    }

    GLFWmonitor* monitor_to_use = glfwGetPrimaryMonitor();

    // 1.e A monitor was specified. Does it exist?
    if (bs_session.config.preferred_monitor[0] != 0) {
        int num_monitors;
        GLFWmonitor** monitors = glfwGetMonitors(&num_monitors);

        bool found = false;

        // 1.e.i Finds the correct monitor to used based on name and size
        for (int i = 0; i < num_monitors; i++) {
            if (strncmp(bs_session.config.preferred_monitor, glfwGetMonitorName(monitors[i]), Config::MAX_PREFERRED_MONITOR_LEN) == 0) {
                monitor_to_use = monitors[i];
                found = true;
                break;
            }
        }

        // 1.e.ii If no monitor is found, we log a warning
        if (!found) logger::log(logger::WARNING, "Preferred monitor %s not found!", bs_session.config.preferred_monitor);
    }

    // 1.f Get the correct resolution for the monitor we want to use.
    const GLFWvidmode* video_mode = glfwGetVideoMode(monitor_to_use);

    // 1.g Create a fullscreen window. Title isn't displayed, so doesn't really matter.
    GLFWwindow* window =
        glfwCreateWindow(video_mode->width, video_mode->height, "Base Station", monitor_to_use, NULL);

    // 1.h Update the window so everyone can access it.
    gui::state.window = window;

    // Set sticky keys mode. It makes our input work as intended.
    // glfwSetInputMode(window, GLFW_STICKY_KEYS, 1);

    // 1.i Disable mouse.
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_HIDDEN);


    glfwSetCharCallback(window, gui::glfw_character_callback);
    glfwSetWindowUserPointer(window, &bs_session);
    glfwSetKeyCallback(window, gui::glfw_key_callback);

    // 1.j Create an OpenGL context.
    glfwMakeContextCurrent(window);

    // 1.k OpenGL Setup.
    glViewport(0, 0, video_mode->width, video_mode->height);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, gui::WINDOW_WIDTH, gui::WINDOW_HEIGHT, 0, 0, 0.5);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // 1.l Initial setup for GUI here so that network errors are printed to log view.
    bs_session.map_texture_id = gui::load_texture("res/binghamton.jpg");
    bs_session.stopwatch_texture_id = gui::load_texture_alpha("res/stopwatch_white.png");

    // 1.m Load this down here so that sizing is correct.
    bool loaded_font = gui::load_font(&gui::state.global_font, "res/FiraMono-Regular.ttf", 100);
    // 1.n If we cannot find the loaded font, log an error message
    if (!loaded_font) {
        logger::log(logger::ERROR, "Failed to load font!");
        return 1;
    }

    logger::register_handler(gui::log_view::log_view_handler);

    // 1.o Initialize camera stuff.
    camera_feed::init();

    // 1.p Create the camera streams.
    for (int i = 0; i < MAX_FEEDS; i++) {
        static char init_feed_name_buffer[camera_feed::FEED_NAME_MAX_LEN + 1];
        snprintf(init_feed_name_buffer, sizeof(init_feed_name_buffer), "UNKNOWN-%d", i);

        camera_feed::init_feed(bs_session.camera_feeds + i, init_feed_name_buffer, 1280, 720);
    }

    // 1.q Initialize network functionality.
    {
        auto err = network::init(
            &bs_session.bs_feed,
            network::FeedType::OUT,
            bs_session.config.interface,
            bs_session.config.base_station_multicast_group,
            bs_session.config.base_station_port,
            &bs_session.global_clock);

        // 1.q.i If we cannot init the base station feed, we log an error
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

        // 1.q.ii If we cannot init the rover feed, log an error
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
        // 1.q.iii If we cannot init the video feed, we log an error
        if (err != network::Error::OK) {
            logger::log(logger::ERROR, "Failed to init video feed: %s", network::get_error_string(err));
            return 1;
        }
        logger::log(
            logger::INFO,
            "Network: subscribed to video computer feed on %s:%d",
            bs_session.config.video_multicast_group,
            bs_session.config.video_port);

        
    }

    // 1.r Init last movement info timers.
    util::Timer::init(&bs_session.movement_send_timer, MOVEMENT_SEND_INTERVAL, &bs_session.global_clock);
    util::Timer::init(&bs_session.arm_send_timer, ARM_SEND_INTERVAL, &bs_session.global_clock);
    util::Timer::init(&bs_session.network_stats_timer, NETWORK_STATS_INTERVAL, &bs_session.global_clock);

    // 1.s Init the controller.
    // TODO: QUERY /sys/class/input/js1/device/id/{vendor,product} TO FIND THE RIGHT CONTROLLER.
    if (controller::init("/dev/input/js0") == controller::Error::OK) {
        bs_session.controller_loaded = true;
        logger::log(logger::INFO, "Controller connected.");
    } else {
        logger::log(logger::WARNING, "No controller connected!");
    }

    gui::debug_console::log("Debug log initialized.", 0, 1.0, 0);

    bool help_menu_up = false;
    bool stopwatch_menu_up = false;

    // 1.t Add the help menu commands here
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

    // 1.u Main loop for the basestation
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        bool z_on = glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS;

        // 1.u.i If z and the up arrow are pressed, zoom in on the waypoint map
        if (z_on && (glfwGetKey(window,GLFW_KEY_UP) == GLFW_PRESS)) {
            gui::waypoint_map::zoom_in();    
        } 
        // 1.u.ii If z and the down arrow are pressed, zoom in on the waypoint map
        else if (z_on && (glfwGetKey(window,GLFW_KEY_DOWN) == GLFW_PRESS)) {
            gui::waypoint_map::zoom_out();
        } 
        // 1.u.iii If the up arrow is pressed and the input state is key command
        else if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS && gui::state.input_state == gui::InputState::KEY_COMMAND) {
            // 1.u.iii.1 If shift is pressed, move to the top of the log, else move up one line
            if (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS) {
                gui::log_view::moveTop();
            } else {
                gui::log_view::moveUpOne();
            }
        } 
        // 1.u.iv If the down arrow is pressed and the input state is key command
        else if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS && gui::state.input_state == gui::InputState::KEY_COMMAND) {
            // 1.u.iv.1 If shift is pressed, move to the bottom of the log, else move down one line
            if (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS) {
                gui::log_view::moveBottom();
            } else {
                gui::log_view::moveDownOne();
            }
        } 
        // 1.u.v If the q key is pressed
        else if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) {
            // 1.u.v.1 If left control is also pressed, exit the program
            if (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS) {
                break;
            }
        } 
        // 1.u.vi If the h key is pressed
        else if (glfwGetKey(window, GLFW_KEY_H) == GLFW_PRESS) {
            // 1.u.vi.1 If the input state is key command, bring up the help menu
            if (gui::state.input_state == gui::InputState::KEY_COMMAND) help_menu_up = true;
        } 
        // 1.u.vii If the escape key is pressed
        else if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
            // 1.u.vii.1 If the help menu is up, close it
            if (help_menu_up) help_menu_up = false;
            // 1.u.vii.2 If the stop watch menu is up, close it and set the input state to key command
            if (stopwatch_menu_up) {
                stopwatch_menu_up = false;
                gui::state.input_state = gui::InputState::KEY_COMMAND;
            }
        } 
        // 1.u.viii If the s key is pressed and the input state is key command, show the stopwatch menu
        else if (
            glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS && gui::state.input_state == gui::InputState::KEY_COMMAND) {
            stopwatch_menu_up = true;
            gui::state.input_state = gui::InputState::STOPWATCH_MENU;
        }
        // 1.u.ix If the input state is stopwatch menu
        if (gui::state.input_state == gui::InputState::STOPWATCH_MENU) {
            // 1.u.ix.1 If the space key is pressed, toggle the stopwatch state based on current state
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
            } 
            // 1.u.ix.2 If the r key is pressed, reset the time
            else if (glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS) {
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

        // 1.u.x Handle incoming network messages from the rover feed.
        {
            network::IncomingMessage message;
            // 1.u.x.1 Parses a full message from the subsystem computer
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

                // 1.u.x.1.a Switches how messages are handled based on the message type
                switch (message.type) {
                    case network::MessageType::LIDAR: {
                        bs_session.lidar_points.clear();

                        network::LidarMessage lidar_message;
                        network::deserialize(&message.buffer, &lidar_message);

                        // 1.u.x.1.a.i Adds lidar points from the message to the base station
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
        // 1.u.xi Handle incoming network messages from the video computer feed.
        {
            network::IncomingMessage message;
            // 1.u.xi.1 Parses a full message from the video computer
            while (true) {
                auto neterr = network::receive(&bs_session.v_feed, &message);
                // 1.u.xi.1.a If the video computer returned an error
                if (neterr != network::Error::OK) {
                    // 1.u.xi.1.a.i If there are no more messages to parse, break the loop
                    if (neterr == network::Error::NOMORE) {
                        break;
                    } 
                    // 1.u.xi.1.a.ii If a different error occured, log the error
                    else {
                        logger::log(
                            logger::WARNING,
                            "Failed to read network packets: %s",
                            network::get_error_string(neterr));
                        break;
                    }
                }

                // 1.u.xi.1.b Swtiches the handling of messages based on the type
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

                        // 1.u.xi.1.b.i If there is a camrea feed error, log it
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

        // 1.u.xii Update network stuff.
        network::update_status(&bs_session.r_feed);
        network::update_status(&bs_session.bs_feed);
        network::update_status(&bs_session.v_feed);

        // 1.u.xiii If we network stats are ready, we can set them in the basestation
        if (bs_session.network_stats_timer.ready()) {
            bs_session.r_tp = (float) bs_session.r_feed.bytes_transferred / ((float) NETWORK_STATS_INTERVAL * 1000.0f);
            bs_session.bs_tp = (float) bs_session.bs_feed.bytes_transferred / ((float) NETWORK_STATS_INTERVAL * 1000.0f);
            bs_session.v_tp = (float) bs_session.v_feed.bytes_transferred / ((float) NETWORK_STATS_INTERVAL * 1000.0f);
            bs_session.t_tp = bs_session.r_tp + bs_session.bs_tp + bs_session.v_tp;

            bs_session.r_feed.bytes_transferred = 0;
            bs_session.bs_feed.bytes_transferred = 0;
            bs_session.v_feed.bytes_transferred = 0;
        }

        // 1.u.xiv Process controller input.
        if (bs_session.controller_loaded) {
            controller::Event event;
            controller::Error err;

            // 1.u.xiv.1 While the controller has no errors
            while ((err = controller::poll(&event)) == controller::Error::OK) {
                // 1.u.xiv.1.a If a button is pressed
                if (event.type == controller::EventType::BUTTON) {
                    // 1.u.xiv.1.a.i If the back button is pressed, log a debug message and swap the feeds
                    if (event.button == controller::Button::BACK && event.value != 0) {
                        logger::log(logger::DEBUG, "button");
                        int temp = bs_session.primary_feed;
                        bs_session.primary_feed = bs_session.secondary_feed;
                        bs_session.secondary_feed = temp;
                    } 
                    // 1.u.xiv.1.a.ii If the xbox button is pressed, swap controller modes
                    else if (event.button == controller::Button::XBOX && event.value != 0) {
                        switch (bs_session.controller_mode) {
                            case ControllerMode::DRIVE:
                                bs_session.controller_mode = ControllerMode::ARM;
                                break;
                            case ControllerMode::ARM:
                                bs_session.controller_mode = ControllerMode::DRIVE;
                                break;
                            case ControllerMode::SCIENCE:
                                break;
                            default:
                                break;
                        }
                    }
                }

                // 1.u.xiv.1.b Swaps the way events are handled based on the controller mode
                switch (bs_session.controller_mode) {
                    case ControllerMode::DRIVE:
                        controller::handle_drive_controller_event(event, &bs_session);
                        break;
                    case ControllerMode::ARM:
                        controller::handle_arm_controller_event(event, &bs_session);
                        break;
                    case ControllerMode::SCIENCE:
                        controller::handle_science_controller_event(event, &bs_session);
                        break;
                    default:
                        break;
                }
            }

            // 1.u.xiv.2 If we cannot read from the controller, disconnect
            if (err != controller::Error::DONE) {
                logger::log(logger::ERROR, "Failed to read from the controller! Disabling until reconnect.");
                bs_session.controller_loaded = false;
            }
        }

        // 1.u.xv Allows for controller reconnects
        if(!bs_session.controller_loaded){
            if (controller::init("/dev/input/js0") == controller::Error::OK) {
                bs_session.controller_loaded = true;
                logger::log(logger::INFO, "Controller reconnected.");
            }  
        }

        // 1.u.xvi Publishes a movement message to the network
        if (bs_session.movement_send_timer.ready()) {
            // printf("sending movement with %d, %d\n", last_movement_message.left, last_movement_message.right);
            network::publish(&bs_session.bs_feed, &bs_session.last_movement_message);
        }

        // 1.u.xvii Update and draw GUI.
        gui::do_gui(&bs_session);

        // 1.u.xviii Exports data to the log file
        if (bs_session.log_file.is_open() && bs_session.log_interval_timer.ready()) {
            bs_session.export_data();
        }

        // 1.u.xix If the help menu is enabled, draw it
        if (help_menu_up) gui::do_help_menu(commands, debug_commands, &bs_session);

        // 1.u.xx If the stopwatch is enabled, draw it
        if (stopwatch_menu_up) {
            gui::do_stopwatch_menu(&bs_session);
        }

        glColor4f(0.0f, 0.0f, 0.0f, 1.0f);

        // 1.u.xxi Display our buffer.
        glfwSwapBuffers(window);
    }

    // 1.v Cleanup.
    glfwTerminate();

    return 0;
}
