#include "session.hpp"
#include <cstring>

Session::Session(){
    this->power_inited = false;
    this->suspension_inited = false;
    this->arm_inited = false;
    this->science_inited = false;
    this->gripper_inited = false;
    this->lidar_inited = false;
    this->gps_inited = false;
    this->ticks = 0;
}

Session::~Session() {}

void Session::load_config(const char* filename) {
    sc::SimpleConfig* sc_config;

    auto err = sc::parse(filename, &sc_config);
    if (err != sc::Error::OK) {
        logger::log(logger::ERROR, "Failed to parse config file: %s", sc::get_error_string(sc_config, err));
        exit(1);
    }

    char* rover_port = sc::get(sc_config, "rover_port");
    if (!rover_port) {
        logger::log(logger::ERROR, "Config file missing 'rover_port'!");
        exit(1);
    }
    this->config.rover_port = atoi(rover_port);

    char* base_station_port = sc::get(sc_config, "base_station_port");
    if (!base_station_port) {
        logger::log(logger::ERROR, "Config file missing 'base_station_port'!");
        exit(1);
    }
    this->config.base_station_port = atoi(base_station_port);

    char* video_port = sc::get(sc_config, "video_port");
    if (!video_port) {
        logger::log(logger::ERROR, "Config file missing 'video_port'!");
        exit(1);
    }
    this->config.video_port = atoi(video_port);

    char* base_station_multicast_group = sc::get(sc_config, "base_station_multicast_group");
    if (!base_station_multicast_group) {
        logger::log(logger::ERROR, "Config file missing 'base_station_multicast_group'!");
        exit(1);
    }
    strncpy(this->config.base_station_multicast_group, base_station_multicast_group, 16);

    char* rover_multicast_group = sc::get(sc_config, "rover_multicast_group");
    if (!rover_multicast_group) {
        logger::log(logger::ERROR, "Config file missing 'rover_multicast_group'!");
        exit(1);
    }
    strncpy(this->config.rover_multicast_group, rover_multicast_group, 16);

    char* interface = sc::get(sc_config, "interface");
    if (!interface) {
        // Default.
        strncpy(this->config.interface, "0.0.0.0", 16);
    } else {
        strncpy(this->config.interface, interface, 16);
    }

    char* video_multicast_group = sc::get(sc_config, "video_multicast_group");
    if (!video_multicast_group) {
        logger::log(logger::ERROR, "Config file missing 'video_multicast_group'!");
        exit(1);
    }
    strncpy(this->config.video_multicast_group, video_multicast_group, 16);

    char* gps_serial_id = sc::get(sc_config, "gps_serial_id");
    if (!gps_serial_id) {
        logger::log(logger::ERROR, "Config file missing 'gps_serial_id'!");

        exit(1);
    }
    this->config.gps_serial_id = strdup(gps_serial_id);

    sc::free(sc_config);
}

//void Session::stderr_handler(logger::Level level, std::string message) {
//    fprintf(stderr, "%s\n", message.c_str());
//}