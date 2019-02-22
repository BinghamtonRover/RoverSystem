#include "controller.hpp"

#include <fcntl.h> 
#include <unistd.h>
#include <linux/joystick.h>

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
static int16_t recalibrate(int16_t axis_value){
    int16_t abs_value = std::abs(axis_value);
    if (abs_value < MIN_POS) {
        return 0;
    }
    float percentage = (float(MAX_POS)-0)/(MAX_POS-MIN_POS);
    int16_t  new_axis_value = percentage*(abs_value-MIN_POS);
    if (axis_value < 0){
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
    return button_values[(int)button];
}

int16_t get_value(Axis axis) {
    return axis_values[(int)axis];
}

} // namespace controller