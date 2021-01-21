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

    //Initialize subsystem info registers
    this->drive_sub_init();
    this->arm_sub_init();
    this->drive_sub_init();
    this->science_sub_init();
    this->autonomy_sub_init();
}

Session::~Session() {};

void Session::load_config(const char* filename) {
    sc::SimpleConfig* sc_config;

    auto err = sc::parse(filename, &sc_config);
    if (err != sc::Error::OK) {
        logger::log(logger::ERROR, "Failed to parse config file: %s", sc::get_error_string(sc_config, err));
        if (err == sc::Error::FILE_OPEN) {
            logger::log(logger::ERROR, "(Did you forget to run the program from the repo root?)");
        }
        exit(1);
    }

    //Configure ports
    char* rover_port = sc::get(sc_config, "rover_port");
    if (!rover_port) {
        logger::log(logger::ERROR, "Config missing 'rover_port'!");
        exit(1);
    }
    this->config.rover_port = atoi(rover_port);

    char* base_station_port = sc::get(sc_config, "base_station_port");
    if (!base_station_port) {
        logger::log(logger::ERROR, "Config missing 'base_station_port'!");
        exit(1);
    }
    this->config.base_station_port = atoi(base_station_port);

    char* video_port = sc::get(sc_config, "video_port");
    if (!video_port) {
        logger::log(logger::ERROR, "Config missing 'video_port'!");
        exit(1);
    }
    this->config.video_port = atoi(video_port);

    //Configure multicast group 
    char* rover_multicast_group = sc::get(sc_config, "rover_multicast_group");
    if (!rover_multicast_group) {
        logger::log(logger::ERROR, "Config missing 'rover_multicast_group'!");
        exit(1);
    }
    strncpy(this->config.rover_multicast_group, rover_multicast_group, 16);

    char* base_station_multicast_group = sc::get(sc_config, "base_station_multicast_group");
    if (!base_station_multicast_group) {
        logger::log(logger::ERROR, "Config missing 'base_station_multicast_group'!");
        exit(1);
    }
    strncpy(this->config.base_station_multicast_group, base_station_multicast_group, 16);

    char* video_multicast_group = sc::get(sc_config, "video_multicast_group");
    if (!video_multicast_group) {
        logger::log(logger::ERROR, "Config missing 'video_multicast_group'!");
        exit(1);
    }
    strncpy(this->config.video_multicast_group, video_multicast_group, 16);

    //Configure base station interface
    char* interface = sc::get(sc_config, "interface");
    if (!interface) {
        // Default.
        strncpy(this->config.interface, "0.0.0.0", 16);
    } else {
        strncpy(this->config.interface, interface, 16);
    }

    char* preferred_monitor = sc::get(sc_config, "preferred_monitor");
    if (!preferred_monitor) {
        // Default: empty string means primary monitor.
        this->config.preferred_monitor[0] = 0;
    } else {
        strncpy(this->config.preferred_monitor, preferred_monitor, Config::MAX_PREFERRED_MONITOR_LEN + 1);
    }

    sc::free(sc_config);
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

void Session::update_focus_mode(int input_mode){
    switch (input_mode)
    {
    case (int)FocusMode::GENERAL:
        this->focus_mode = FocusMode::GENERAL;
        break;
    case (int)FocusMode::DRIVE:
        this->focus_mode = FocusMode::DRIVE;
        break;
    case (int)FocusMode::ARM:
        this->focus_mode = FocusMode::ARM;
        break;
    case (int)FocusMode::SCIENCE:
        this->focus_mode = FocusMode::SCIENCE;
        break;
    case (int)FocusMode::AUTONOMY:
        this->focus_mode = FocusMode::AUTONOMY;
        break;
    default:
    logger::log(logger::ERROR, "Base Station: Invalid Focus Mode.");
        break;
    }
}

void Session::drive_sub_init(){
    std::pair<std::string, double> speed_stat;
    speed_stat.first = "Speed (km/s)";
    speed_stat.second = 0.0;
    this->drive_sub_info.insert(speed_stat);

    std::pair<std::string, double> direc_stat;
    direc_stat.first = "Direction";
    direc_stat.second = 0.0;
    this->drive_sub_info.insert(direc_stat);

    std::pair<std::string, double> accel_stat;
    accel_stat.first = "Acceleration (m/s^2)";
    accel_stat.second = 0.0;
    this->drive_sub_info.insert(accel_stat);

    std::pair<std::string, double> distt_stat;
    distt_stat.first = "Distance Traveled (meters)";
    distt_stat.second = 0.0;
    this->drive_sub_info.insert(distt_stat);

    std::pair<std::string, double> distw_stat;
    distw_stat.first = "Distance to Waypoint (meters)";
    distw_stat.second = 0.0;
    this->drive_sub_info.insert(distw_stat);

    std::pair<std::string, double> motor_stat;
    motor_stat.first = "Motor Current (Amps)";
    motor_stat.second = 0.0;
    this->drive_sub_info.insert(motor_stat);
}

void Session::arm_sub_init(){
    std::pair<std::string, double> xloca_stat;
    xloca_stat.first = "X-Location (cm)";
    xloca_stat.second = 0.0;
    this->arm_sub_info.insert(xloca_stat);

    std::pair<std::string, double> yloca_stat;
    yloca_stat.first = "Y-Location (cm)";
    yloca_stat.second = 0.0;
    this->arm_sub_info.insert(yloca_stat);

    std::pair<std::string, double> zloca_stat;
    zloca_stat.first = "Z-Location (cm)";
    zloca_stat.second = 0.0;
    this->arm_sub_info.insert(zloca_stat);

    std::pair<std::string, double> grpos_stat;
    grpos_stat.first = "Gripper Position (deg)";
    grpos_stat.second = 0.0;
    this->arm_sub_info.insert(grpos_stat);
}

void Session::science_sub_init(){
    std::pair<std::string, double> tempr_stat;
    tempr_stat.first = "Temperature (C)";
    tempr_stat.second = 0.0;
    this->science_sub_info.insert(tempr_stat);

    std::pair<std::string, double> metha_stat;
    metha_stat.first = "Methane Levels (ppm)";
    metha_stat.second = 0.0;
    this->science_sub_info.insert(metha_stat);

    std::pair<std::string, double> co2cn_stat;
    co2cn_stat.first = "CO2 Concentration (ppm)";
    co2cn_stat.second = 0.0;
    this->science_sub_info.insert(co2cn_stat);

    std::pair<std::string, double> humid_stat;
    humid_stat.first = "Humidity (%)";
    humid_stat.second = 0.0;
    this->science_sub_info.insert(humid_stat);

    std::pair<std::string, double> pHlev_stat;
    pHlev_stat.first = "pH";
    pHlev_stat.second = 0.0;
    this->science_sub_info.insert(pHlev_stat);
}

void Session::autonomy_sub_init(){
    std::pair<std::string, double> arloc_stat;
    arloc_stat.first = "AR Marker Lock";
    arloc_stat.second = 0.0;
    this->autonomy_sub_info.insert(arloc_stat);

    std::pair<std::string, double> distw_stat;
    distw_stat.first = "Distance to Waypoint (meters)";
    distw_stat.second = 0.0;
    this->autonomy_sub_info.insert(distw_stat);
}

void Session::start_log(const char* filename) {
    log_file.open(filename);
    if (log_file.is_open()) {
        log_file << "Timestamp,";
        auto itr = science_sub_info.begin();
        while (itr != science_sub_info.end()) {
            log_file << itr->first;
            ++itr;
            if (itr != science_sub_info.end()) {
                log_file << ',';
            }
        }
        log_file << std::endl;
    }
}

void Session::stop_log() {
    log_file.close();
}

void Session::export_data() {
    auto itr = science_sub_info.begin();
    while (itr != science_sub_info.end()) {
        log_file << itr->second;
        ++itr;
        if (itr != science_sub_info.end()) {
            log_file << ',';
        }
    }
    log_file << std::endl;
}