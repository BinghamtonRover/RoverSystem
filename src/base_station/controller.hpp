#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <cstdint>
#include "session.hpp"

namespace controller {

// Button mapping for the XBox One controller.
enum class Button : uint16_t {
    A = 0,
    B = 1,
    X = 2,
    Y = 3,
    LB = 4,
    RB = 5,
    BACK = 6,
    START = 7,
    XBOX = 8,
    JS_LEFT = 9,
    JS_RIGHT = 10
};
const int NUM_BUTTONS = 11;

// Axis mapping for the Xbox One controller.
enum class Axis : uint16_t {
    JS_LEFT_X = 0,
    JS_LEFT_Y = 1,
    JS_RIGHT_X = 3,
    JS_RIGHT_Y = 4,
    DPAD_X = 6,
    DPAD_Y = 7,
    LT = 2,
    RT = 5
};
const int NUM_AXES = 8;

// Represents the different error states associated with reading from the controller.
enum class Error {
    OK,
    DONE,

    OPEN,
    READ
};

enum class EventType { BUTTON, AXIS, OTHER };

struct Event {
    EventType type;

    union {
        Button button;
        Axis axis;
    };

    int16_t value;
};

// Initializes the global controller by attempting to open the given device file.
// Returns Error::OK on success and Error::OPEN if the file cannot be opened.
Error init(const char* device_file);

// Polls for joystick events, filling in the given event.
// Returns Error::OK if there are more events to be polled.
// Returns Error::DONE if there are no more events to process
// (if Error::DONE is returned, then no event was found).
// Returns Error::READ if reading from the device file fails.
Error poll(Event* event);

// Retrieves the value of the given button or axis.
// 0 is returned if no update has been received.
int16_t get_value(Button button);
int16_t get_value(Axis axis);

float smooth_rover_input(float value);
void handle_drive_controller_event(controller::Event event, Session* bs_session);
void handle_arm_controller_event(controller::Event event, Session* bs_session);

} // namespace controller
#endif