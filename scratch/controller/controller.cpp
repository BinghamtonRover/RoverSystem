#include "../../src/network/network.hpp"
#include <chrono>
#include <cmath>
#include <fcntl.h>
#include <iostream>
#include <linux/joystick.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

const unsigned short
    // Button values
    A_BUTTON = 0,
    B_BUTTON = 1, X_BUTTON = 2, Y_BUTTON = 3, LB_BUTTON = 4, RB_BUTTON = 5, BACK_BUTTON = 6, START_BUTTON = 7,
    HOME_BUTTON = 9,
    // Axis values
    JOYSTICK_LEFT_X = 0, JOYSTICK_LEFT_Y = 1, JOYSTICK_RIGHT_X = 3, JOYSTICK_RIGHT_Y = 4, DPAD_X = 6, DPAD_Y = 7,
    LT_AXIS = 2, RT_AXIS = 5;
// Axis ranges from -32767 to +32767
// If recalibrate is used, the input will be read between
// POSITIVE: MIN_POS and MAX_POS
// NEGATIVE: -MAX_POS and -MIN_POS
// and then scaled to fit range -32767 to 32767.
// Essentially making MIN_POS and less the new 0 or 'dead' value.
//-3000 to 3000 seems like a good dead range.
const int16_t MAX_POS = 32767, MIN_POS = 3000;
// Fixed controller update rate
const auto RATE = std::chrono::milliseconds(100);
const char *device_name = "/dev/input/js0";
struct movement
{
    int16_t left, right;
};
// Joystick values range from -32767 to +32767
// Used formula scaled_val = (new_max - new_min) / (old_max - old_min) * (v - old_min) + new_min
int16_t recalibrate(int16_t axis_value)
{
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

int main(int argc, char *argv[])
{
    network::Connection conn;
    network::Error error;
    if ((error = network::connect(&conn, "192.168.1.1", 45545, 45545)) != network::Error::OK) {
        fprintf(stderr, "[!] Failed to connect to rover!\n");
        return 1;
    }
    network::MovementMessage currentMovement = {0};
    network::Buffer *outgoing;
    // Device input defaults to js0
    if (argc == 2) {
        device_name = argv[1];
    }
    // Controller file descriptor. O_NONBLOCK mdoe ensures multiple events can
    // be read and added to diver queue
    int fd = open(device_name, O_RDONLY | O_NONBLOCK);
    if (fd == -1) {
        fprintf(stderr, "[!] Failed to find controller!\n");
        return 1;
    }
    struct js_event event;
    auto lastUpdated = std::chrono::system_clock::now();
    auto now = std::chrono::system_clock::now();
    // Main loop
    while (true) {
        // Loop until event queue is empty, resulting in -1
        while (read(fd, &event, sizeof(event)) > 0) {
            unsigned short button_number;
            switch (event.type) {
                    // Unused button event handler
                case JS_EVENT_BUTTON:
                    break;
                // Only care about the left and right Y values for axis events
                case JS_EVENT_AXIS:
                    if (event.number == JOYSTICK_LEFT_Y) {
                        currentMovement.left = recalibrate(event.value);
                    } else if (event.number == JOYSTICK_RIGHT_Y) {
                        currentMovement.right = recalibrate(event.value);
                    }
                    break;
                default:
                    // Ignore init events
                    break;
            }
        }
        // End program if controller is disconnected
        if (errno != EAGAIN) {
            fprintf(stderr, "[!] Controller disconnected\n");
            return 1;
        }
        now = std::chrono::system_clock::now();
        // Push currentMovement to the outgoing buffer if >= RATE has passed
        // since the last push. now-lastUpdated is an std::chrono::duration.
        // The difference is evaluated as ms.
        if (now - lastUpdated >= RATE) {
            outgoing = network::get_outgoing_buffer();
            network::serialize(outgoing, &currentMovement);
            network::queue_outgoing(&conn, network::MessageType::MOVEMENT, outgoing);
            lastUpdated = now;
        }
        network::drain_outgoing(&conn);
    }
}
