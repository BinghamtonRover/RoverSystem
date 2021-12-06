#include "session.hpp"

int main() {
    Session subsys_session;
    logger::register_handler(logger::stderr_handler);
    util::Clock::init(&subsys_session.global_clock);
    subsys_session.load_config("res/r.sconfig");
    logger::log(logger::INFO, "Initializing Subsystem Computer...");

    // i2c
    // goodbye I2C <3 - JM
    /*
    if (rocs::init("/dev/i2c-1") != rocs::Error::OK) {
        logger::log(logger::ERROR, "[!] Failed to init ROCS!");
        return 1;
    }
    */

    // Suspension subsystem and CAN bus init
    for (int i = 0; i < SUSPENSION_CONNECT_TRIES; i++) {
        if (suspension::init_calibrate() != 0) {
            logger::log(logger::WARNING, "[!] Failed to init suspension and CAN!");
            //logger::log(logger::WARNING, "[!] Failed to init suspension (try %d).", i);
        } 
        else {
            subsys_session.suspension_inited = true;
            break;
        }
    }
    if (!subsys_session.suspension_inited) {
        logger::log(logger::ERROR, "[!] Failed to start suspension!");
        //return 1;
    }

    // Arm subsystem init
    for (int i = 0; i < SUSPENSION_CONNECT_TRIES; i++) {
        if (arm::init(ARM_I2C_ADDR) != arm::Error::OK) {
            logger::log(logger::WARNING, "[!] Failed to init arm (try %d).", i);
        } 
        else {
            subsys_session.arm_inited = true;
            break;
        }
    }
    if (!subsys_session.arm_inited) {
        logger::log(logger::ERROR, "[!] Failed to start arm!");
        //return 1;
    }

    // LIDAR init
    if (lidar::start("192.168.1.21") != lidar::Error::OK) {
        logger::log(logger::ERROR, "[!] Failed to start lidar!");
        //subsys_session.lidar_inited = false;
        //return 1;
    }
    else {
        subsys_session.lidar_inited = true;
    }

    // GPS init
    if (gps::init(subsys_session.config.gps_serial_id, &subsys_session.global_clock) != gps::Error::OK) {
        logger::log(logger::ERROR, "[!] Failed to open GPS!");
        //subsys_session.gps_inited = false;
        //return 1;
    }
    else {
        subsys_session.gps_inited = true;
    }
    
    // Three feeds: incoming base station and video computer; outgoing rover.
    {
        auto err = network::init(
            &subsys_session.r_feed,
            network::FeedType::OUT,
            subsys_session.config.interface,
            subsys_session.config.rover_multicast_group,
            subsys_session.config.rover_port,
            &subsys_session.global_clock);

        if (err != network::Error::OK) {
            logger::log(logger::ERROR, "[!] Failed to start rover feed: %s", network::get_error_string(err));
            exit(1);
        }
    }

    {
        auto err = network::init(
            &subsys_session.bs_feed,
            network::FeedType::IN,
            subsys_session.config.interface,
            subsys_session.config.base_station_multicast_group,
            subsys_session.config.base_station_port,
            &subsys_session.global_clock);

        if (err != network::Error::OK) {
            logger::log(logger::ERROR, "[!] Failed to subscribe to base station feed: %s", network::get_error_string(err));
            exit(1);
        }
    }

    {
        auto err = network::init(
            &subsys_session.v_feed,
            network::FeedType::IN,
            subsys_session.config.interface,
            subsys_session.config.video_multicast_group,
            subsys_session.config.video_port,
            &subsys_session.global_clock);

        if (err != network::Error::OK) {
            logger::log(logger::ERROR, "[!] Failed to subscribe to video computer feed: %s", network::get_error_string(err));
            exit(1);
        }
    }

    util::Timer::init(&subsys_session.location_send_timer,LOCATION_SEND_INTERVAL, &subsys_session.global_clock);
    util::Timer::init(&subsys_session.tick_timer, TICK_INTERVAL, &subsys_session.global_clock);
    util::Timer::init(&subsys_session.network_update_timer, NETWORK_UPDATE_INTERVAL, &subsys_session.global_clock);
    util::Timer::init(&subsys_session.subsystem_send_timer, SUBSYSTEM_SEND_INTERVAL, &subsys_session.global_clock);
    util::Timer::init(&subsys_session.suspension_update_timer, SUSPENSION_UPDATE_INTERVAL, &subsys_session.global_clock);
    util::Timer::init(&subsys_session.arm_update_timer, ARM_UPDATE_INTERVAL, &subsys_session.global_clock);
    util::Timer::init(&subsys_session.lidar_update_timer, LIDAR_UPDATE_INTERVAL, &subsys_session.global_clock);

    util::Timer::init(&subsys_session.power_read_data, POWER_READ_DATA_INTERVAL, &subsys_session.global_clock);
    util::Timer::init(&subsys_session.suspension_read_data, SUSPENSION_READ_DATA_INTERVAL, &subsys_session.global_clock);
    util::Timer::init(&subsys_session.arm_read_data, ARM_READ_DATA_INTERVAL, &subsys_session.global_clock);
    util::Timer::init(&subsys_session.science_read_data, SCIENCE_READ_DATA_INTERVAL, &subsys_session.global_clock);

    suspension::set_target_speed(255, 255, 0.0f);
    // Subsystem computer main loop
    while (true) {
        if (subsys_session.location_send_timer.ready() && subsys_session.gps_inited) {
            network::LocationMessage message;

            auto fix = gps::get_fix();
            message.fix_status = fix;

            auto pos = gps::get_position();
            message.latitude = pos.latitude;
            message.longitude = pos.longitude;

            network::publish(&subsys_session.r_feed,&message);
        }

        if (subsys_session.lidar_update_timer.ready() && subsys_session.lidar_inited) {
            network::LidarMessage message;

            subsys_session.lidar_points.clear();
            if (lidar::scan(subsys_session.lidar_points) != lidar::Error::OK) {
                logger::log(logger::ERROR, "Lidar scan failed");
            } 
            else {
                for (int i = 0; i < network::LidarMessage::NUM_LIDAR_POINTS; i++) {
                    message.points[i] = (uint16_t) subsys_session.lidar_points[i];
                }
                network::publish(&subsys_session.r_feed, &message);
            }
        }

        // Receive incoming messages
        network::IncomingMessage message;

        while (true) {
            auto neterr = network::receive(&subsys_session.bs_feed, &message);
            if (neterr != network::Error::OK) {
                if (neterr == network::Error::NOMORE) {
                    break;
                }
                logger::log(logger::DEBUG, "Network error on receive!");
                break;
            }
            switch (message.type) {
                case network::MessageType::MOVEMENT: {
                    network::deserialize(&message.buffer, &subsys_session.last_movement_message);
                    break;
                }
                case network::MessageType::ARM: {
                    network::deserialize(&message.buffer, &subsys_session.last_arm_message);
                    //logger::log(logger::DEBUG, "Joint: %d, Movement: %d", (int)subsys_session.last_arm_message.joint, (int)subsys_session.last_arm_message.movement);
                    break;
                }
                case network::MessageType::FOCUS_MODE: {
                    network::FocusModeMessage focus_mode_message;
                    network::deserialize(&message.buffer, &focus_mode_message);
                    subsys_session.subsys_focus_mode = focus_mode_message.focus_mode;
                    // Echo back to BS to verify mode change.
                    network::publish(&subsys_session.r_feed, &focus_mode_message);
                    break;
                }
                default:
                    break;
            }
        }

        // Update network status
        if (subsys_session.network_update_timer.ready()) {
            // Update feed statuses.
            network::update_status(&subsys_session.r_feed);
            network::update_status(&subsys_session.bs_feed);
            network::update_status(&subsys_session.v_feed);
        }

        // Update suspension subsystem speed
        if (subsys_session.suspension_update_timer.ready() && subsys_session.suspension_inited) {
            auto movement = subsys_session.last_movement_message;
            logger::log(logger::DEBUG, "Received left: %d; right: %d", movement.left, movement.right);

            suspension::set_target_speed(movement.left, movement.right, subsys_session.global_clock.get_millis());
        }
        else if (subsys_session.suspension_inited) {
            suspension::update(subsys_session.global_clock.get_millis());
        }

        if(subsys_session.power_read_data.ready() && subsys_session.power_inited){
            
        }

        if(subsys_session.suspension_read_data.ready() && subsys_session.suspension_inited){
            
        }

        if(subsys_session.arm_read_data.ready() && subsys_session.arm_inited){
            
        }

        if(subsys_session.science_read_data.ready() && subsys_session.science_inited){
            
        }
        if (subsys_session.arm_update_timer.ready()) {
            auto movement = subsys_session.last_arm_message;          
            if (arm::update(static_cast<network::ArmMessage::Joint>(movement.joint),
                static_cast<network::ArmMessage::Movement>(movement.movement)) != arm::Error::OK) {
                    logger::log(logger::ERROR, "Failed to update arm");
            }
        }

        // Tick.
        subsys_session.ticks++;
        uint32_t last_tick_interval;
        if (subsys_session.tick_timer.ready(&last_tick_interval)) {
            network::TickMessage message;
            message.ticks_per_second = (subsys_session.ticks * TICK_INTERVAL) / (float)last_tick_interval;
            network::publish(&subsys_session.r_feed, &message);

            subsys_session.ticks = 0;
        }

        if (subsys_session.subsystem_send_timer.ready()) {
            subsystem::send_update(&subsys_session.r_feed);
        }
    }
}
