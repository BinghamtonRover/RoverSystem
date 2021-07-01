#include "controller.hpp"
#include "../logger/logger.hpp"

#include <fcntl.h>
#include <linux/joystick.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>

namespace controller {

// As of now, we only support a single controller.
// If there is a need to support more than one, factor
// this global state into a struct.
int global_controller_fd = -1;

// Axis ranges from -32767 to +32767
// If recalibrate is used, the input will be read between
// POSITIVE: MIN_POS and MAX_POS
// NEGATIVE: -MAX_POS and -MIN_POS
// and then scaled to fit range -32767 to 32767.
// Essentially making MIN_POS and less the new 0 or 'dead' value.
// -3000 to 3000 seems like a good dead range.
constexpr int16_t MAX_POS = 32767, MIN_POS = 3000;

// Global containers for current button and axis values.
int16_t button_values[NUM_BUTTONS]{};
int16_t axis_values[NUM_AXES]{};

// Joystick values range from -32767 to +32767
// Used formula scaled_val = (new_max - new_min) / (old_max - old_min) * (v - old_min) + new_min
static int16_t recalibrate(int16_t axis_value) {
    int16_t abs_value = std::abs(axis_value);

    //Rounds down values to create the deadzone of values 0-3000
    if (abs_value < MIN_POS) {
        return 0;
    }

    float percentage = (float(MAX_POS) - 0) / (MAX_POS - MIN_POS);
    int16_t new_axis_value = percentage * (abs_value - MIN_POS);

    //Returns correctly signed recalibrated value
    if (axis_value < 0) {
        return -new_axis_value;
    }
    return new_axis_value;
};

//Inites controller fom given device file
Error init(const char* device_file) {
    global_controller_fd = open(device_file, O_RDONLY | O_NONBLOCK);

    //If the controller failes to open from the file, return an error
    if (global_controller_fd == -1) {
        return Error::OPEN;
    }

    return Error::OK;
}

//Polls the device for input
Error poll(Event* event) {
    struct js_event linux_event;

    int bread = read(global_controller_fd, &linux_event, sizeof(linux_event));

    //If device is unavailable or unable to be read, return an error
    if (bread <= 0) {
        if (errno == EAGAIN) {
            return Error::DONE;
        }

        return Error::READ;
    }

    //Creates events based on the linux event tyoe
    switch (linux_event.type) {
        case JS_EVENT_BUTTON:
            event->type = EventType::BUTTON;
            event->button = (Button) linux_event.number;
            event->value = linux_event.value;

            button_values[linux_event.number] = event->value;
            break;
        case JS_EVENT_AXIS:
            event->type = EventType::AXIS;
            event->axis = (Axis) linux_event.number;
            event->value = recalibrate(linux_event.value);

            axis_values[linux_event.number] = event->value;
            break;
        default:
            // We don't care about other events.
            event->type = EventType::OTHER;
            break;
    }

    return Error::OK;
}

//Gets the value from the input button
int16_t get_value(Button button) {
    return button_values[(int) button];
}

//Gets the value from the axis
int16_t get_value(Axis axis) {
    return axis_values[(int) axis];
}

// Takes values between 0 and 255 and returns them between 0 and 255.
float smooth_rover_input(float value) {
    // We want to exponentially smooth this.
    // We do that by picking alpha in (1, whatever).
    // The higher the alpha, the more exponential the thing.
    // We then make sure that f(0) = 0 and f(1) = 255.

    return (255.0f / (CONTROL_ALPHA - 1)) * (powf(1 / CONTROL_ALPHA, -value / 255.0f) - 1);
}

//Parses drive control event and updates the movement message
void handle_drive_controller_event(Event event, Session* bs_session) {
    //Ensures movement mesage uses the axis
    if (event.type == EventType::AXIS) {
        bool forward;
        (event.value <= 0) ? forward = true : forward = false;
        int16_t abs_val = event.value < 0 ? -event.value : event.value;
        int16_t smoothed = (int16_t) smooth_rover_input((float) (abs_val >> 7));
        //Handles movement to the left
        if (event.axis == Axis::JS_LEFT_Y) {
            bs_session->last_movement_message.left = smoothed;
            //If not forward, reverses smoothed value
            if(!forward){
                bs_session->last_movement_message.left *= -1;
            }
            //logger::log(logger::DEBUG, "Left orig: %d, smooth: %d", abs_val, bs_session->last_movement_message.left);
        }
        //Handles movements to the right
        if (event.axis == Axis::JS_RIGHT_Y) {
            bs_session->last_movement_message.right = smoothed;
            //If not forward, reverses smoothed value
            if(!forward){
                bs_session->last_movement_message.right *= -1;
            }
            //logger::log(logger::DEBUG, "Right orig: %d, smooth: %d", abs_val, bs_session->last_movement_message.right);
        }
    } 
}

//Handles events dealing with the arm
void handle_arm_controller_event(Event event, Session* bs_session) {
    //Handles button inputs for the arm
    if (event.type == EventType::BUTTON) {
        //Based on the button pressed, changes the region of being controlled
        switch (event.button) {
            case Button::A:
                bs_session->arm_control_region = ArmControlRegion::GRIPPER;
                break;
            case Button::B:
                bs_session->arm_control_region = ArmControlRegion::WRIST;
                break;
            case Button::X:
                bs_session->arm_control_region = ArmControlRegion::SHOULDER;
                break;
            case Button::Y:
                bs_session->arm_control_region = ArmControlRegion::ELBOW;
                break;
            default:
                return;
        }
    } 

    //Handles axis inputs for the arm
    else if (event.type == EventType::AXIS) {
        //X axis rotates, Y axis change joint angles
        switch (event.axis) {
            case Axis::DPAD_X:
                //Checks if the control region is shoulder
                if(bs_session->arm_control_region == ArmControlRegion::SHOULDER){
                    //Values less than zero rotate counter clockwise, greater than zero are clockwise, zero stops rotation
                    if(event.value < 0){
                        bs_session->last_arm_message.joint = (int16_t)network::ArmMessage::Joint::BASE_ROTATE;
                        bs_session->last_arm_message.movement = (int16_t)network::ArmMessage::Movement::COUNTER;
                        logger::log(logger::DEBUG, "Base rotate left");
                    }
                    else if(event.value > 0){
                        bs_session->last_arm_message.joint = (int16_t)network::ArmMessage::Joint::BASE_ROTATE;
                        bs_session->last_arm_message.movement = (int16_t)network::ArmMessage::Movement::CLOCK;
                        logger::log(logger::DEBUG, "Base rotate right");
                    }
                    else{
                        bs_session->last_arm_message.joint = (int16_t)network::ArmMessage::Joint::BASE_ROTATE;
                        bs_session->last_arm_message.movement = (int16_t)network::ArmMessage::Movement::STOP;
                        logger::log(logger::DEBUG, "Base no rotation");
                    }
                }

                //Checks if control region is gripper
                if(bs_session->arm_control_region == ArmControlRegion::GRIPPER){
                    //Values less than zero rotate counter clockwise, greater than zero are clockwise, zero stops rotation
                    if(event.value < 0){
                        bs_session->last_arm_message.joint = (int16_t)network::ArmMessage::Joint::GRIPPER_ROTATE;
                        bs_session->last_arm_message.movement = (int16_t)network::ArmMessage::Movement::COUNTER;
                        logger::log(logger::DEBUG, "Gripper rotate left");
                    }
                    else if(event.value > 0){
                        bs_session->last_arm_message.joint = (int16_t)network::ArmMessage::Joint::GRIPPER_ROTATE;
                        bs_session->last_arm_message.movement = (int16_t)network::ArmMessage::Movement::CLOCK;
                        logger::log(logger::DEBUG, "Gripper rotate right");
                    }
                    else{
                        bs_session->last_arm_message.joint = (int16_t)network::ArmMessage::Joint::GRIPPER_ROTATE;
                        bs_session->last_arm_message.movement = (int16_t)network::ArmMessage::Movement::STOP;
                        logger::log(logger::DEBUG, "Gripper no rotation");
                    }
                }
                break;
            case Axis::DPAD_Y:
                //Checks if control region is shoulder
                if(bs_session->arm_control_region == ArmControlRegion::SHOULDER){
                    //Negative value moves base up, positive moves base down, zero keeps base stationary
                    if(event.value < 0){
                        bs_session->last_arm_message.joint = (int16_t)network::ArmMessage::Joint::BASE_SHOULDER;
                        bs_session->last_arm_message.movement = (int16_t)network::ArmMessage::Movement::COUNTER;
                        logger::log(logger::DEBUG, "Base up");
                    }
                    else if(event.value > 0){
                        bs_session->last_arm_message.joint = (int16_t)network::ArmMessage::Joint::BASE_SHOULDER;
                        bs_session->last_arm_message.movement = (int16_t)network::ArmMessage::Movement::CLOCK;
                        logger::log(logger::DEBUG, "Base down");
                    }
                    else{
                        bs_session->last_arm_message.joint = (int16_t)network::ArmMessage::Joint::BASE_SHOULDER;
                        bs_session->last_arm_message.movement = (int16_t)network::ArmMessage::Movement::STOP;
                        logger::log(logger::DEBUG, "Base still");
                    }
                }

                //Checks if control region is elbow
                if(bs_session->arm_control_region == ArmControlRegion::ELBOW){
                    //Negative value moves elbow up, positive moves elbow down, zero keeps elbow stationary
                    if(event.value < 0){
                        bs_session->last_arm_message.joint = (int16_t)network::ArmMessage::Joint::ELBOW;
                        bs_session->last_arm_message.movement = (int16_t)network::ArmMessage::Movement::COUNTER;
                        logger::log(logger::DEBUG, "Elbow up");
                    }
                    else if(event.value > 0){
                        bs_session->last_arm_message.joint = (int16_t)network::ArmMessage::Joint::ELBOW;
                        bs_session->last_arm_message.movement = (int16_t)network::ArmMessage::Movement::CLOCK;
                        logger::log(logger::DEBUG, "Elbow down");
                    }
                    else{
                        bs_session->last_arm_message.joint = (int16_t)network::ArmMessage::Joint::ELBOW;
                        bs_session->last_arm_message.movement = (int16_t)network::ArmMessage::Movement::STOP;
                        logger::log(logger::DEBUG, "Elbow still");
                    }
                }

                //Checks if control region is wrist
                if(bs_session->arm_control_region == ArmControlRegion::WRIST){
                    //Negative value moves wrist up, positive moves wrist down, zero keeps wrist stationary
                    if(event.value < 0){
                        bs_session->last_arm_message.joint = (int16_t)network::ArmMessage::Joint::WRIST;
                        bs_session->last_arm_message.movement = (int16_t)network::ArmMessage::Movement::COUNTER;
                        logger::log(logger::DEBUG, "Wrist up");
                    }
                    else if(event.value > 0){
                        bs_session->last_arm_message.joint = (int16_t)network::ArmMessage::Joint::WRIST;
                        bs_session->last_arm_message.movement = (int16_t)network::ArmMessage::Movement::CLOCK;
                        logger::log(logger::DEBUG, "Wrist down");
                    }
                    else{
                        bs_session->last_arm_message.joint = (int16_t)network::ArmMessage::Joint::WRIST;
                        bs_session->last_arm_message.movement = (int16_t)network::ArmMessage::Movement::STOP;
                        logger::log(logger::DEBUG, "Wrist still");
                    }
                }

                //Checks if control region is gripper
                if(bs_session->arm_control_region == ArmControlRegion::GRIPPER){
                    //Negative value opens gripper up, positive moves closes gripper, zero keeps gripper stationary
                    if(event.value < 0){
                        bs_session->last_arm_message.joint = (int16_t)network::ArmMessage::Joint::GRIPPER_FINGERS;
                        bs_session->last_arm_message.movement = (int16_t)network::ArmMessage::Movement::COUNTER;
                        logger::log(logger::DEBUG, "Gripper opening");
                    }
                    else if(event.value > 0){
                        bs_session->last_arm_message.joint = (int16_t)network::ArmMessage::Joint::GRIPPER_FINGERS;
                        bs_session->last_arm_message.movement = (int16_t)network::ArmMessage::Movement::CLOCK;
                        logger::log(logger::DEBUG, "Gripper closing");
                    }
                    else{
                        bs_session->last_arm_message.joint = (int16_t)network::ArmMessage::Joint::GRIPPER_FINGERS;
                        bs_session->last_arm_message.movement = (int16_t)network::ArmMessage::Movement::STOP;
                        logger::log(logger::DEBUG, "Gripper still");
                    }
                }
                break;
            default:
                return;
        }
    }
}

//Handles events upder the science mode
void handle_science_controller_event(Event event, Session* bs_session) {
    //Checks what mode the science mode is in
    if(bs_session->science_mode == ScienceMode::DIRT_COLLECTION_MODE){
        //Ckecks for button presses or axis movements
        if(event.type == EventType::BUTTON){
            //Switches based on button pressed
            switch (event.button) {
                case Button::LB:
                    //When pressed, move button to tests, else it remains stationary
                    if(event.value == 1){
                        logger::log(logger::DEBUG, "Carousel to Tests");
                    }
                    else {
                        logger::log(logger::DEBUG, "Stop Carousel");
                    }
                    break;
                case Button::RB:
                    //When pressed, move button to auger, else it remains stationary
                    if(event.value == 1){
                        logger::log(logger::DEBUG, "Carousel to Auger");
                    }
                    else {
                        logger::log(logger::DEBUG, "Stop Carousel");
                    }
                    break;
                case Button::XBOX:
                    //Switches mode to testing mode
                    bs_session->science_mode = ScienceMode::TESTING_MODE;
                    break;
                default:
                    break;
            }
        }
        else if(event.type == EventType::AXIS){
            //Switches based on axis
            switch (event.axis) {
                case Axis::DPAD_X:
                    //Negative values CCW Auger, Positive values CW Auger, else it stops auger
                    if(event.value < 0){
                        logger::log(logger::DEBUG, "CCW Auger");
                    }
                    else if(event.value > 0){
                        logger::log(logger::DEBUG, "CW Auger");
                    }
                    else{
                        logger::log(logger::DEBUG, "Stop Auger");
                    }
                    break;
                case Axis::DPAD_Y:
                    //Negative values lowers auger, Positive values raises Auger, else it stops auger
                    if(event.value < 0){
                        logger::log(logger::DEBUG, "Lower Auger");
                    }
                    else if(event.value > 0){
                        logger::log(logger::DEBUG, "Raise Auger");
                    }
                    else{
                        logger::log(logger::DEBUG, "Still Auger");
                    }
                    break;
                default:
                    break;
            }
        }
    }
    else if(bs_session->science_mode == ScienceMode::TESTING_MODE){
        //Checks if event type is button or axis
        if(event.type == EventType::BUTTON){
            switch (event.button) {
                case Button::X:
                    //X presses run pump 1, x releases stop the pump
                    if(event.value == 1){
                        logger::log(logger::DEBUG, "Run Pump 1");
                    }
                    else {
                        logger::log(logger::DEBUG, "Stop Pump 1");
                    }
                    break;
                case Button::Y:
                    //Y presses run pump 2, y releases stop the pump
                    if(event.value == 1){
                        logger::log(logger::DEBUG, "Run Pump 2");
                    }
                    else {
                        logger::log(logger::DEBUG, "Stop Pump 2");
                    }
                    break;
                case Button::B:
                    //B presses run pump 3, b releases stop the pump
                    if(event.value == 1){
                        logger::log(logger::DEBUG, "Run Pump 3");
                    }
                    else {
                        logger::log(logger::DEBUG, "Stop Pump 3");
                    }
                    break;
                case Button::A:
                    //A presses run pump 4, a releases stop the pump
                    if(event.value == 1){
                        logger::log(logger::DEBUG, "Run Pump 4");
                    }
                    else {
                        logger::log(logger::DEBUG, "Stop Pump 4");
                    }
                    break;
                case Button::XBOX:
                    //Pressing the XBOX button changes the science mode
                    bs_session->science_mode = ScienceMode::DIRT_COLLECTION_MODE;
                    break;
                default:
                    break;
            }
        }
        else if(event.type == EventType::AXIS){
            int16_t smoothed;
            //Switches based on axis
            switch (event.axis) {
                case Axis::DPAD_Y:
                    //Negative lowers tests, positive raises tests, zero keeps tests stationary
                    if(event.value < 0){
                        logger::log(logger::DEBUG, "Lower Tests");
                    }
                    else if(event.value > 0){
                        logger::log(logger::DEBUG, "Raise Tests");
                    }
                    else{
                        logger::log(logger::DEBUG, "Still Tests");
                    }
                    break;
                case Axis::LT:
                    smoothed = (int16_t) smooth_rover_input((float) (event.value >> 7));
                    logger::log(logger::DEBUG, "CCW Carousel: %d", smoothed);
                    break;
                case Axis::RT:
                    smoothed = (int16_t) smooth_rover_input((float) (event.value >> 7));
                    logger::log(logger::DEBUG, "CW Carousel: %d", smoothed);
                    break;
                default:
                    break;
            }
        }
    }
}

} // namespace controller
