#include "session.hpp"



//Create Session instance and initialize variables
Session::Session(){
    this->mode = network::ModeMessage::Mode::MANUAL;
    this->last_rover_tick = 0;

    //Network stats
    this->r_tp = 0;
    this->bs_tp = 0;
    this->t_tp = 0;

    this->last_movement_message = { 0, 0 };

    //These get initialized off-the-bat.
    //We only care about feed_to_move value when we are in camera move mode.
    this->primary_feed = 0;
    this->secondary_feed = 1;
    this->feed_to_move = -1;

    controller_mode = controller::ControllerMode::DRIVE;
}

Session::~Session() {};