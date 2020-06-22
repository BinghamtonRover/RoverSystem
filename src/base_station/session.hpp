#ifndef SESSION
#define SESSION

#include "../network/network.hpp"
#include "gui.hpp"
#include "constant_vars.hpp"
#include "camera_feed.hpp"

struct autonomy_info_struct {
    network::AutonomyStatusMessage::Status status = network::AutonomyStatusMessage::Status::IDLE;
    bool has_target = false;
    float target_lat = 0, target_lon = 0;
    int edit_idx = 0;
    std::string edit_lat, edit_lon;
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
    //START


    //Declares stopwatch
    

    //END

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