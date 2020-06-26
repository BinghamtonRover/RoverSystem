#ifndef SESSION
#define SESSION

#include "../network/network.hpp"
#include "camera_feed.hpp"
#include <string>
#include "stb_truetype.h"
#include "controller.hpp"
#include <vector>

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

const int LOG_VIEW_WIDTH = 572;
const int LOG_VIEW_HEIGHT = 458;

const int PRIMARY_FEED_WIDTH = 1298;
const int PRIMARY_FEED_HEIGHT = 730;

const int SECONDARY_FEED_WIDTH = 533;
const int SECONDARY_FEED_HEIGHT = 300;

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

enum class StopwatchState { STOPPED, PAUSED, RUNNING };

struct StopwatchStruct {
    StopwatchState state;
    unsigned int start_time;
    unsigned int pause_time;
};

struct Font {
    // Information that keeps track of each character that we want to be able to draw.
    stbtt_bakedchar baked_chars[95];

    // The font is one big texture!
    unsigned int texture_id;

    // Maximum height of ASCII characters at the loaded size.
    int max_height;
};


class Session {
private:
public:
    //Variables (Definitions)
    network::ModeMessage::Mode mode;

    autonomy_info_struct autonomy_info;
   
    // Network feeds.cd 
    network::Feed r_feed;
    network::Feed bs_feed;

    Font global_font;

    unsigned int map_texture_id;

    unsigned int stopwatch_texture_id;

    float last_rover_tick;

    //Declares stopwatch
    StopwatchStruct stopwatch;

    //Network stats
    float r_tp;
    float bs_tp; 
    float t_tp;

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

    controller::ControllerMode controller_mode;

    std::vector<uint16_t> lidar_points;

    //Constructor & Destructor
    Session();
    ~Session();
};

#endif