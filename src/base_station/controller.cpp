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
    if (abs_value < MIN_POS) {
        return 0;
    }
    float percentage = (float(MAX_POS) - 0) / (MAX_POS - MIN_POS);
    int16_t new_axis_value = percentage * (abs_value - MIN_POS);
    if (axis_value < 0) {
        return -new_axis_value;
    }
    return new_axis_value;
};

Error init(const char* device_file) {
    global_controller_fd = open(device_file, O_RDONLY | O_NONBLOCK);
    if (global_controller_fd == -1) {
        return Error::OPEN;
    }

    return Error::OK;
}

Error poll(Event* event) {
    struct js_event linux_event;

    int bread = read(global_controller_fd, &linux_event, sizeof(linux_event));

    if (bread <= 0) {
        if (errno == EAGAIN) {
            return Error::DONE;
        }

        return Error::READ;
    }

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

int16_t get_value(Button button) {
    return button_values[(int) button];
}

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

void handle_drive_controller_event(Event event, Session* bs_session) {
    if (event.type == EventType::AXIS) {
        bool forward;
        (event.value <= 0) ? forward = true : forward = false;
        int16_t abs_val = event.value < 0 ? -event.value : event.value;
        int16_t smoothed = (int16_t) smooth_rover_input((float) (abs_val >> 7));
        if (event.axis == Axis::JS_LEFT_Y) {
            bs_session->last_movement_message.left = smoothed;
            if(!forward){
                bs_session->last_movement_message.left *= -1;
            }
            //logger::log(logger::DEBUG, "Left orig: %d, smooth: %d", abs_val, bs_session->last_movement_message.left);
        }
        if (event.axis == Axis::JS_RIGHT_Y) {
            bs_session->last_movement_message.right = smoothed;
            if(!forward){
                bs_session->last_movement_message.right *= -1;
            }
            //logger::log(logger::DEBUG, "Right orig: %d, smooth: %d", abs_val, bs_session->last_movement_message.right);
        }
    } 
}

void handle_arm_controller_event(Event event, Session* bs_session) {
    if (event.type == EventType::BUTTON) {
        network::ArmMessage::Motor motor;
        network::ArmMessage::State state;

        switch (event.button) {
            case Button::A:
                motor = network::ArmMessage::Motor::GRIPPER_FINGER;
                state = network::ArmMessage::State::CLOCK;
                break;
            case Button::B:
                motor = network::ArmMessage::Motor::GRIPPER_FINGER;
                state = network::ArmMessage::State::COUNTER;
                break;
            case Button::X:
                motor = network::ArmMessage::Motor::GRIPPER_WRIST_ROTATE;
                state = network::ArmMessage::State::CLOCK;
                break;
            case Button::Y:
                motor = network::ArmMessage::Motor::GRIPPER_WRIST_ROTATE;
                state = network::ArmMessage::State::COUNTER;
                break;
            case Button::LB:
                motor = network::ArmMessage::Motor::GRIPPER_WRIST_FLEX;
                state = network::ArmMessage::State::CLOCK;
                break;
            case Button::RB:
                motor = network::ArmMessage::Motor::GRIPPER_WRIST_FLEX;
                state = network::ArmMessage::State::COUNTER;
                break;
            default:
                return;
        }

        if (event.value == 0) state = network::ArmMessage::State::STOP;

        bs_session->last_arm_message.set_state(motor, state);
    } else if (event.type == EventType::AXIS) {
        network::ArmMessage::Motor motor;
        network::ArmMessage::State state;

        switch (event.axis) {
            case Axis::DPAD_X:
                motor = network::ArmMessage::Motor::ARM_LOWER;
                if (event.value >= 0) {
                    state = network::ArmMessage::State::CLOCK;
                } else {
                    state = network::ArmMessage::State::COUNTER;
                }
                break;
            case Axis::DPAD_Y:
                motor = network::ArmMessage::Motor::ARM_UPPER;
                if (event.value >= 0) {
                    state = network::ArmMessage::State::CLOCK;
                } else {
                    state = network::ArmMessage::State::COUNTER;
                }
                break;
            case Axis::LT:
                motor = network::ArmMessage::Motor::ARM_BASE;
                if (event.value >= INT16_MAX / 2) {
                    state = network::ArmMessage::State::COUNTER;
                } else {
                    state = network::ArmMessage::State::STOP;
                }
                break;
            case Axis::RT:
                motor = network::ArmMessage::Motor::ARM_BASE;
                if (event.value >= INT16_MAX / 2) {
                    state = network::ArmMessage::State::CLOCK;
                } else {
                    state = network::ArmMessage::State::STOP;
                }
                break;
            default:
                return;
        }

        if (event.value == 0) state = network::ArmMessage::State::STOP;

        bs_session->last_arm_message.set_state(motor, state);
    }
}

} // namespace controller