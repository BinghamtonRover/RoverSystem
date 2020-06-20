#include "session.hpp"

//Create Session instance and initialize variables
Session::Session(){
    mode = network::ModeMessage::Mode::MANUAL;
}

Session::~Session() {};