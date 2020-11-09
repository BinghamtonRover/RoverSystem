//#include "../network/network.hpp"
//#include "../simple_config/simpleconfig.h"
//#include "../util/util.hpp"
//#include "../logger/logger.hpp"
#include "../rocs/rocs.hpp"

#include "session.hpp"

#include "gps.hpp"
#include "subsystem.hpp"
#include "suspension.hpp"
#include "lidar.hpp"
#include "gripper.hpp"
#include "arm.hpp"


//#include <cstddef>
//#include <cstdio>
//#include <cstdlib>
//#include <cstring>
//#include <iostream>
//#include <vector>

// GLOBAL CONSTANTS

// ROCS addresses.
//const uint8_t SUSPENSION_I2C_ADDR = 0x01;
//const uint8_t ARM_I2C_ADDR = 0x02;
//const uint8_t GRIPPER_I2C_ADDR = 0x03;
//
//const int SUSPENSION_UPDATE_INTERVAL = 1000 / 15;
//
//const int SUSPENSION_CONNECT_TRIES = 5;
//
//const int NETWORK_UPDATE_INTERVAL = 1000 / 2;
//
//const int SUBSYSTEM_SEND_INTERVAL = 1000 * 5;
//
//const int LOCATION_SEND_INTERVAL = 1000;
//
//const int LIDAR_UPDATE_INTERVAL = 1000 / 15;
//
//const int TICK_INTERVAL = 1000;
//
//network::ModeMessage::Mode mode = network::ModeMessage::Mode::MANUAL;
//
//util::Clock global_clock;
//
//struct Config
//{
//    int base_station_port;
//    int rover_port;
//
//    char base_station_multicast_group[16];
//    char rover_multicast_group[16];
//    char interface[16];
//
//    // For now, this is dynamically-sized.
//    char* gps_serial_id;
//};
//
//Config load_config(const char* filename) {
//    Config config;
//
//    sc::SimpleConfig* sc_config;
//
//    auto err = sc::parse(filename, &sc_config);
//    if (err != sc::Error::OK) {
//        logger::log(logger::ERROR, "Failed to parse config file: %s", sc::get_error_string(sc_config, err));
//        exit(1);
//    }
//
//    char* rover_port = sc::get(sc_config, "rover_port");
//    if (!rover_port) {
//        logger::log(logger::ERROR, "Config file missing 'rover_port'!");
//        exit(1);
//    }
//    config.rover_port = atoi(rover_port);
//
//    char* base_station_port = sc::get(sc_config, "base_station_port");
//    if (!base_station_port) {
//        logger::log(logger::ERROR, "Config file missing 'base_station_port'!");
//        exit(1);
//    }
//    config.base_station_port = atoi(base_station_port);
//
//    char* base_station_multicast_group = sc::get(sc_config, "base_station_multicast_group");
//    if (!base_station_multicast_group) {
//        logger::log(logger::ERROR, "Config file missing 'base_station_multicast_group'!");
//        exit(1);
//    }
//    strncpy(config.base_station_multicast_group, base_station_multicast_group, 16);
//
//    char* rover_multicast_group = sc::get(sc_config, "rover_multicast_group");
//    if (!rover_multicast_group) {
//        logger::log(logger::ERROR, "Config file missing 'rover_multicast_group'!");
//        exit(1);
//    }
//    strncpy(config.rover_multicast_group, rover_multicast_group, 16);
//
//    char* interface = sc::get(sc_config, "interface");
//    if (!interface) {
//        // Default.
//        strncpy(config.interface, "0.0.0.0", 16);
//    } else {
//        strncpy(config.interface, interface, 16);
//    }
//
//    char* gps_serial_id = sc::get(sc_config, "gps_serial_id");
//    if (!gps_serial_id) {
//        logger::log(logger::ERROR, "Config file missing 'gps_serial_id'!");
//
//        exit(1);
//    }
//    config.gps_serial_id = strdup(gps_serial_id);
//
//    sc::free(sc_config);
//
//    return config;
//}

//void stderr_handler(logger::Level leve, std::string message) {
//    fprintf(stderr, "%s\n", message.c_str());
//}

int main() {
    Session subsys_session;
    logger::register_handler(logger::stderr_handler);
    util::Clock::init(&subsys_session.global_clock);
    subsys_session.load_config("res/r.sconfig");
    logger::log(logger::INFO, "Initializing Subsystem Computer...");


    //logger::register_handler(stderr_handler);

    //util::Clock::init(&global_clock);

    //Config config = load_config("res/r.sconfig");

    //unsigned int frame_counter = 0;

    // i2c
    if (rocs::init("/dev/i2c-1") != rocs::Error::OK) {
        logger::log(logger::ERROR, "[!] Failed to init ROCS!");
        return 1;
    }

    //bool suspension_inited = false;
    for (int i = 0; i < SUSPENSION_CONNECT_TRIES; i++) {
        if (suspension::init(SUSPENSION_I2C_ADDR) != suspension::Error::OK) {
            logger::log(logger::WARNING, "[!] Failed to init suspension (try %d).", i);
        } else {
            subsys_session.suspension_inited = true;
            break;
        }
    }
    if (!subsys_session.suspension_inited) {
        logger::log(logger::ERROR, "[!] Failed to start suspension!");
        return 1;
    }

    //bool arm_inited = false;
    for (int i = 0; i < SUSPENSION_CONNECT_TRIES; i++) {
        if (arm::init(ARM_I2C_ADDR) != arm::Error::OK) {
            logger::log(logger::WARNING, "[!] Failed to init arm (try %d).", i);
        } else {
            subsys_session.arm_inited = true;
            break;
        }
    }
    if (!subsys_session.arm_inited) {
        logger::log(logger::ERROR, "[!] Failed to start arm!");
        return 1;
    }

    //bool gripper_inited = false;
    for (int i = 0; i < SUSPENSION_CONNECT_TRIES; i++) {
        if (gripper::init(GRIPPER_I2C_ADDR) != gripper::Error::OK) {
            logger::log(logger::WARNING, "[!] Failed to init gripper (try %d).", i);
        } else {
            subsys_session.gripper_inited = true;
            break;
        }
    }
    if (!subsys_session.gripper_inited) {
        logger::log(logger::ERROR, "[!] Failed to start gripper!");
        return 1;
    }

    if (lidar::start("192.168.1.21") != lidar::Error::OK) {
        logger::log(logger::ERROR, "[!] Failed to start lidar!");
        return 1;
    }

    if (gps::init(subsys_session.config.gps_serial_id, &subsys_session.global_clock) != gps::Error::OK) {
        logger::log(logger::ERROR, "[!] Failed to open GPS!");
        return 1;
    }

    // Two feeds: incoming base station and outgoing rover.
    //network::Feed r_feed, bs_feed;

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

    //util::Timer location_send_timer;
    util::Timer::init(&subsys_session.location_send_timer,LOCATION_SEND_INTERVAL, &subsys_session.global_clock);

    //util::Timer tick_timer;
    util::Timer::init(&subsys_session.tick_timer, TICK_INTERVAL, &subsys_session.global_clock);

    //util::Timer network_update_timer;
    util::Timer::init(&subsys_session.network_update_timer, NETWORK_UPDATE_INTERVAL, &subsys_session.global_clock);

    //util::Timer subsystem_send_timer;
    util::Timer::init(&subsys_session.subsystem_send_timer, SUBSYSTEM_SEND_INTERVAL, &subsys_session.global_clock);

    //util::Timer suspension_update_timer;
    util::Timer::init(&subsys_session.suspension_update_timer, SUSPENSION_UPDATE_INTERVAL, &subsys_session.global_clock);

    //util::Timer lidar_update_timer;
    util::Timer::init(&subsys_session.lidar_update_timer, LIDAR_UPDATE_INTERVAL, &subsys_session.global_clock);

    //uint32_t ticks = 0;

    //network::MovementMessage last_movement_message = { 0, 0 };

    //std::vector<long> lidar_points;

    logger::log(logger::INFO, "Success! Entering main loop.");
    while (true) {

        if (subsys_session.location_send_timer.ready()) {
            network::LocationMessage message;

            auto fix = gps::get_fix();
            message.fix_status = fix;

            auto pos = gps::get_position();
            message.latitude = pos.latitude;
            message.longitude = pos.longitude;


            network::publish(&subsys_session.r_feed,&message);
        }

        if (subsys_session.lidar_update_timer.ready()) {
            network::LidarMessage message;

            subsys_session.lidar_points.clear();
            if (lidar::scan(subsys_session.lidar_points) != lidar::Error::OK) {
                logger::log(logger::ERROR, "Lidar scan failed");
            } else {
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
                    network::ArmMessage arm_message;
                    network::deserialize(&message.buffer, &arm_message);

                    arm::update(network::ArmMessage::Motor::ARM_LOWER, arm_message.states[static_cast<uint8_t>(network::ArmMessage::Motor::ARM_LOWER)]);
                    arm::update(network::ArmMessage::Motor::ARM_UPPER, arm_message.states[static_cast<uint8_t>(network::ArmMessage::Motor::ARM_UPPER)]);
                    arm::update(network::ArmMessage::Motor::ARM_BASE, arm_message.states[static_cast<uint8_t>(network::ArmMessage::Motor::ARM_BASE)]);
                    
                    gripper::update(network::ArmMessage::Motor::GRIPPER_FINGER, arm_message.states[static_cast<uint8_t>(network::ArmMessage::Motor::GRIPPER_FINGER)]);
                    gripper::update(network::ArmMessage::Motor::GRIPPER_WRIST_ROTATE, arm_message.states[static_cast<uint8_t>(network::ArmMessage::Motor::GRIPPER_WRIST_ROTATE)]);
                    gripper::update(network::ArmMessage::Motor::GRIPPER_WRIST_FLEX, arm_message.states[static_cast<uint8_t>(network::ArmMessage::Motor::GRIPPER_WRIST_FLEX)]);
                    /*
                    logger::log(logger::DEBUG, "arm update:");
                    logger::log(logger::DEBUG, "  gfinger=%d", static_cast<uint8_t>(arm_message.get_state(network::ArmMessage::Motor::GRIPPER_FINGER)));
                    logger::log(logger::DEBUG, "  gwrotate=%d", static_cast<uint8_t>(arm_message.get_state(network::ArmMessage::Motor::GRIPPER_WRIST_ROTATE)));
                    logger::log(logger::DEBUG, "  gwflex=%d", static_cast<uint8_t>(arm_message.get_state(network::ArmMessage::Motor::GRIPPER_WRIST_FLEX)));
                    logger::log(logger::DEBUG, "  arml=%d", static_cast<uint8_t>(arm_message.get_state(network::ArmMessage::Motor::ARM_LOWER)));
                    logger::log(logger::DEBUG, "  armu=%d", static_cast<uint8_t>(arm_message.get_state(network::ArmMessage::Motor::ARM_UPPER)));
                    logger::log(logger::DEBUG, "  armb=%d", static_cast<uint8_t>(arm_message.get_state(network::ArmMessage::Motor::ARM_BASE)));
                    */
                    break;
                }
                case network::MessageType::MODE: {
                    network::ModeMessage mode_message;
                    network::deserialize(&message.buffer, &mode_message);
                    subsys_session.mode = mode_message.mode;
                    // Echo back to BS to verify mode change.
                    network::publish(&subsys_session.r_feed, &mode_message);
                }
                default:
                    break;
            }
        }

        if (subsys_session.network_update_timer.ready()) {
            // Update feed statuses.
            network::update_status(&subsys_session.r_feed);
            network::update_status(&subsys_session.bs_feed);
        }

        if (subsys_session.suspension_update_timer.ready()) {
            auto movement = subsys_session.last_movement_message;

            suspension::Direction left_dir;
            suspension::Direction right_dir;

            if (movement.left < 0) {
                left_dir = suspension::Direction::BACKWARD;
                movement.left = -movement.left;
            } else if (movement.left > 0) {
                left_dir = suspension::Direction::FORWARD;
            }

            if (movement.left > 255) movement.left = 255;

            if (movement.right < 0) {
                right_dir = suspension::Direction::BACKWARD;
                movement.right = -movement.right;
            } else if (movement.right > 0) {
                right_dir = suspension::Direction::FORWARD;
            } 

            if (movement.right > 255) movement.right = 255;

            if (movement.left == 0) {
                if (suspension::stop(suspension::Side::LEFT) != suspension::Error::OK) {
                    logger::log(logger::ERROR, "Failed to stop left suspension side");
                }
            } else {
                if (suspension::update(suspension::Side::LEFT, left_dir, (uint8_t)movement.left) != suspension::Error::OK) {
                    logger::log(logger::ERROR, "Failed to update left suspension side");
                }
            }

            if (movement.right == 0) {
                if (suspension::stop(suspension::Side::RIGHT) != suspension::Error::OK) {
                    logger::log(logger::ERROR, "Failed to stop right suspension side");
                }
            } else {
                if (suspension::update(suspension::Side::RIGHT, right_dir, (uint8_t)movement.right) != suspension::Error::OK) {

                    logger::log(logger::ERROR, "Failed to update right suspension side");
                }
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
