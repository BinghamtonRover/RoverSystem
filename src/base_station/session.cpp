#include "session.hpp"

//Create Session instance and initialize variables
Session::Session(){
    mode = network::ModeMessage::Mode::MANUAL;
    last_rover_tick = 0;

    //Network stats
    r_tp = 0;
    bs_tp = 0;
    t_tp = 0;

    last_movement_message = { 0, 0 };

    //These get initialized off-the-bat.
    //We only care about feed_to_move value when we are in camera move mode.
    primary_feed = 0;
    secondary_feed = 1;
    feed_to_move = -1;

    controller_mode = controller::ControllerMode::DRIVE;
}

Session::~Session() {};