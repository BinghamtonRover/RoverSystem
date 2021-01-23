#ifndef SESSION
#define SESSION

#include "../network/network.hpp"
#include "../simple_config/simpleconfig.h"
#include "shared_feeds.hpp" 
#include "camera_feed.hpp"
#include "stb_truetype.h"

#include <string>
#include <vector>
#include <cstring>
#include <unordered_map>

// Default angular resolution (vertices / radian) to use when drawing circles.
constexpr float ANGULAR_RES = 10.0f;

// We know that our base station will have this resolution.
const int WINDOW_WIDTH = 1920;
const int WINDOW_HEIGHT = 1080;

// For control smoothing.
const float CONTROL_ALPHA = 30;

// Speed for the DPAD up/down.
const int16_t JOINT_DRIVE_SPEED = 100;

// Send movement updates x times per second.
const int MOVEMENT_SEND_INTERVAL = 1000 / 15;

// Update network statistics once per second.
const int NETWORK_STATS_INTERVAL = 1000;

const int ARM_SEND_INTERVAL = 1000 / 9;

const int MAX_FEEDS = 9;

struct autonomy_info_struct {
    network::AutonomyStatusMessage::Status status;
    bool has_target;
    float target_lat;
    float target_lon;
    int edit_idx;
    std::string edit_lat;
    std::string edit_lon;
};

enum class ControllerMode {
    DRIVE,
    ARM
};

enum class FocusMode{
    GENERAL,
    DRIVE,
    ARM,
    SCIENCE,
    AUTONOMY
};

enum class StopwatchState { 
    STOPPED,
    PAUSED,
    RUNNING };

struct StopwatchStruct {
    StopwatchState state;
    unsigned int start_time;
    unsigned int pause_time;
};

struct Config {
    int rover_port;
    int base_station_port;
    int video_port;

    // Max length of string ipv4 addr is 15, plus one for nt.
    char rover_multicast_group[16]; 
    char base_station_multicast_group[16];
    char video_multicast_group[16];
    char interface[16];

    static const int MAX_PREFERRED_MONITOR_LEN = 32;
    char preferred_monitor[MAX_PREFERRED_MONITOR_LEN + 1];
};

class Session {
private:
public:
    //Variables (Definitions)
    network::FocusModeMessage::FocusMode subsystem_focus_mode;
    network::FocusModeMessage::FocusMode video_focus_mode;

    autonomy_info_struct autonomy_info;
   
    // Network feeds.cd 
    network::Feed r_feed;
    network::Feed bs_feed;
    network::Feed v_feed;

    //Network stats
    float r_tp;
    float bs_tp; 
    float v_tp;
    float t_tp;

    //Initialize Focus Mode
    FocusMode bs_focus_mode = FocusMode::GENERAL;

    unsigned int map_texture_id;

    unsigned int stopwatch_texture_id;

    // Rover TPS registers
    float last_subsystem_tick;
    float last_video_tick;

    //Declares stopwatch
    StopwatchStruct stopwatch;

    // Clock!
    util::Clock global_clock;

    network::MovementMessage last_movement_message;

    network::ArmMessage last_arm_message;

    // Camera stuff: These get initialized off-the-bat.
    // We only care about feed_to_move value when we are in camera move mode.
    camera_feed::Feed camera_feeds[MAX_FEEDS];
    int primary_feed;
    int secondary_feed;
    int feed_to_move;

    bool controller_loaded;
    ControllerMode controller_mode;

    std::vector<uint16_t> lidar_points;

    Config config;
    
    //Subsystems information
    std::unordered_map<std::string, double> drive_sub_info; 
    std::unordered_map<std::string, double> arm_sub_info; 
    std::unordered_map<std::string, double> science_sub_info;
    std::unordered_map<std::string, double> autonomy_sub_info;  
    std::unordered_map<std::string, double> power_sub_info;

    //Constructor & Destructor
    Session();
    ~Session();

    void load_config(const char* filename);

    void send_feed(uint8_t stream_indx);
    void send_all_feeds();
    void dont_send_feed(uint8_t stream_indx);
    void dont_send_invalid();
    void update_focus_mode(int input_mode);
    
    void drive_sub_init();
    void arm_sub_init();
    void science_sub_init();
    void autonomy_sub_init();
    void power_sub_init();
};

#endif