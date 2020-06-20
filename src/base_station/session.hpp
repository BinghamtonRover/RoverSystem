#ifndef SESSION
#define SESSION

#include "../network/network.hpp"
#include "gui.hpp"
#include "constant_vars.hpp"

class Session {
private:
public:
    //Variables (Definitions)
    network::ModeMessage::Mode mode;

    gui::autonomy_info_struct autonomy_info;

    // Network feeds.cd 
    network::Feed r_feed;
    network::Feed bs_feed;

    gui::Font global_font;

    unsigned int map_texture_id;

    unsigned int stopwatch_texture_id;

    float last_rover_tick;

    //Declares stopwatch
    gui::StopwatchStruct stopwatch;

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