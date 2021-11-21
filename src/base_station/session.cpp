#include "session.hpp"
#include "../logger/logger.hpp"

#include <algorithm>

//Create Session instance and initialize variables
Session::Session() {
    this->last_subsystem_tick = 0;
    this->last_video_tick = 0;

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
    this->last_arm_message = { 0, 0 };

    //These get initialized off-the-bat.
    //We only care about feed_to_move value when we are in camera move mode.
    this->primary_feed = 0;
    this->secondary_feed = 1;
    this->feed_to_move = -1;

    //Controller info
    this->controller_loaded = false;
    this->controller_mode = ControllerMode::DRIVE;
    this->arm_control_region = ArmControlRegion::SHOULDER;

    //Initialize rover computer default Focus modes
    this->subsystem_focus_mode = network::FocusModeMessage::FocusMode::GENERAL;
    this->video_focus_mode = network::FocusModeMessage::FocusMode::GENERAL;

    //Initialize subsystem info registers
    this->drive_sub_init();
    this->arm_sub_init();
    this->science_sub_init();
    this->autonomy_sub_init();
    this->power_sub_init();
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



void Session::drive_sub_init(){

    keyboard_direction = KeyboardDriveDirection::STOPPED;
    keyboard_steering = KeyboardDriveSteering::STRAIGHT;
    throttle = 0;

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

    std::pair<std::string, double> throttle_stat;
    throttle_stat.first = "Throttle Speed (max 255)";
    throttle_stat.second = 0.0;
    this->drive_sub_info.insert(throttle_stat);

    std::pair<std::string, double> target_right_speed;
    target_right_speed.first = "Target Right Speed";
    target_right_speed.second = 0.0;
    this->drive_sub_info.insert(target_right_speed);

    std::pair<std::string, double> target_left_speed;
    target_left_speed.first = "Target Right Speed";
    target_left_speed.second = 0.0;
    this->drive_sub_info.insert(target_left_speed);

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

    std::pair<std::string, double> atmP_stat;
    atmP_stat.first = "Atmospheric Pressure (bar)";
    atmP_stat.second = 0.0;
    this->science_sub_info.insert(atmP_stat);

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

void Session::power_sub_init(){
    std::pair<std::string, double> battvolt_stat;
    battvolt_stat.first = "Battery Voltage (V)";
    battvolt_stat.second = 0.0;
    this->power_sub_info.insert(battvolt_stat);

    std::pair<std::string, double> enerusag_stat;
    enerusag_stat.first = "Energy Usage (W)";
    enerusag_stat.second = 0.0;
    this->power_sub_info.insert(enerusag_stat);

    std::pair<std::string, double> currtotal_stat;
    currtotal_stat.first = "Curent Total Power Usage (Amps)";
    currtotal_stat.second = 0.0;
    this->power_sub_info.insert(currtotal_stat);

    std::pair<std::string, double> batttemp_stat;
    batttemp_stat.first = "Battery Temperature (C)";
    batttemp_stat.second = 0.0;
    this->power_sub_info.insert(batttemp_stat);    
}

void Session::start_log(const char* filename) {
    log_file.open(filename);
    if (log_file.is_open()) {
        util::Timer::init(&log_interval_timer, 3000, &global_clock);
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
    
    char time_str[24];
    time_t current_time;
    time(&current_time);
    struct tm* time_info = localtime(&current_time);
    strftime(time_str, 24, "%Y-%m-%d_%H:%M:%S.", time_info);
    
    std::string current_millis = std::to_string(global_clock.get_millis() % 1000);
    strcpy(&time_str[20], current_millis.c_str());
    
    log_file << time_str << ',';
    while (itr != science_sub_info.end()) {
        log_file << itr->second;
        ++itr;
        if (itr != science_sub_info.end()) {
            log_file << ',';
        }
    }
    log_file << std::endl;
}

void Session::adjust_throttle(int delta) {
    if (delta > 0) {
        if (MAX_THROTTLE - throttle > delta) {
            throttle += delta;
        } else {
            throttle = MAX_THROTTLE;
        }
    } else {
        if (throttle > -delta) {
            throttle += delta;
        } else {
            throttle = 0;
        }
    }
    drive_sub_info["Throttle Speed (max 255)"] = static_cast<double>(throttle); //we know throttle speed is already initialized in the drive_sub_info   
};

void Session::calc_drive_speed() {
    last_movement_message.left = 0;
    last_movement_message.right = 0;
    if (throttle > 0) {
        bool invert = false;
        if (keyboard_direction == KeyboardDriveDirection::FORWARD) {
            last_movement_message.left = throttle;
            last_movement_message.right = throttle;
        } else if (keyboard_direction == KeyboardDriveDirection::BACKWARD) {
            // Move backward slower than forward
            last_movement_message.left = std::max(throttle / 2, 1);
            last_movement_message.right = std::max(throttle / 2, 1);
            invert = true;
        }

        // For steering, the side we turn to gets slowed down (halved)
        // In the event of a "stationary" turn (no forward/back speed), we must boost the opposite side as well
        if (keyboard_steering == KeyboardDriveSteering::LEFT) {
            last_movement_message.left /= 2;
            last_movement_message.right = std::max({static_cast<int>(last_movement_message.right), throttle / 2, 1});
            
        } else if (keyboard_steering == KeyboardDriveSteering::RIGHT) {
            last_movement_message.right /= 2;
            last_movement_message.left = std::max({static_cast<int>(last_movement_message.left), throttle / 2, 1});
        }

        if (invert) {
            last_movement_message.right = -last_movement_message.right;
            last_movement_message.left = -last_movement_message.left;
        }
    }
}

void Session::axis_forward_speed(float x) {
    
}

void Session::axis_reverse_speed(float x) {

}

void Session::axis_turning(float x) {
    
}
