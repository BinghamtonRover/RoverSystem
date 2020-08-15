#include "session.hpp"
#include "../logger/logger.hpp"

//Create Session instance and initialize variables
Session::Session(){
    this->mode = network::ModeMessage::Mode::MANUAL;
    this->last_rover_tick = 0;

    //Network stats
    this->r_tp = 0;
    this->bs_tp = 0;
    this->t_tp = 0;

    //Initializing autonomy_info struct variables, edit_lat and edit_lon remain uninitialized
    this->autonomy_info.status = network::AutonomyStatusMessage::Status::IDLE;
    this->autonomy_info.has_target = false;
    this->autonomy_info.target_lat = 0;
    this->autonomy_info.target_lon = 0;
    this->autonomy_info.edit_idx = 0;

    this->last_movement_message = { 0, 0 };

    //These get initialized off-the-bat.
    //We only care about feed_to_move value when we are in camera move mode.
    this->primary_feed = 0;
    this->secondary_feed = 1;
    this->feed_to_move = -1;

    //Controller info
    this->controller_loaded = false;
    this->controller_mode = ControllerMode::DRIVE;
}

Session::~Session() {};

Config Session::load_config(const char* filename) {
    Config config;

    sc::SimpleConfig* sc_config;

    auto err = sc::parse(filename, &sc_config);
    if (err != sc::Error::OK) {
        logger::log(logger::ERROR, "Failed to parse config file: %s", sc::get_error_string(sc_config, err));
        if (err == sc::Error::FILE_OPEN) {
            logger::log(logger::ERROR, "(Did you forget to run the program from the repo root?)");
        }
        exit(1);
    }

    char* rover_port = sc::get(sc_config, "rover_port");
    if (!rover_port) {
        logger::log(logger::ERROR, "Config missing 'rover_port'!");
        exit(1);
    }
    config.rover_port = atoi(rover_port);

    char* base_station_port = sc::get(sc_config, "base_station_port");
    if (!base_station_port) {
        logger::log(logger::ERROR, "Config missing 'base_station_port'!");
        exit(1);
    }
    config.base_station_port = atoi(base_station_port);

    char* rover_multicast_group = sc::get(sc_config, "rover_multicast_group");
    if (!rover_multicast_group) {
        logger::log(logger::ERROR, "Config missing 'rover_multicast_group'!");
        exit(1);
    }
    strncpy(config.rover_multicast_group, rover_multicast_group, 16);

    char* base_station_multicast_group = sc::get(sc_config, "base_station_multicast_group");
    if (!base_station_multicast_group) {
        logger::log(logger::ERROR, "Config missing 'base_station_multicast_group'!");
        exit(1);
    }
    strncpy(config.base_station_multicast_group, base_station_multicast_group, 16);

    char* interface = sc::get(sc_config, "interface");
    if (!interface) {
        // Default.
        strncpy(config.interface, "0.0.0.0", 16);
    } else {
        strncpy(config.interface, interface, 16);
    }

    char* preferred_monitor = sc::get(sc_config, "preferred_monitor");
    if (!preferred_monitor) {
        // Default: empty string means primary monitor.
        config.preferred_monitor[0] = 0;
    } else {
        strncpy(config.preferred_monitor, preferred_monitor, Config::MAX_PREFERRED_MONITOR_LEN + 1);
    }

    sc::free(sc_config);

    return config;
}

void Session::send_feed(uint8_t stream_indx) {
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

void Session::send_all_feeds() {
    for(uint8_t i = 0; i < 9; i++) {
        if(i != this->primary_feed && i != this->secondary_feed) {
            this->send_feed(i);
        }
    }
    return;
}

void Session::dont_send_feed(uint8_t stream_indx) {
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

void Session::dont_send_invalid() {
    for(uint8_t i = 0; i < 9; i++) {
        if(i != this->primary_feed && i != this->secondary_feed) {
            this->dont_send_feed(i);
        }
    }
    return;
}
