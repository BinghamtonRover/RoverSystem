#ifndef SUS_H
#define SUS_H

#include <cstdint>
#include "../rocs/rocs.hpp"
#include "../can_bus/can_controller.hpp"
#include <cstring>

#include <iostream>

namespace suspension {

    enum Side : uint8_t { LEFT, RIGHT };

    enum Direction : uint8_t { FORWARD, BACKWARD };

    #define SUSPENSION_ERRORS_DEF(X) \
        X(OK), X(DEVICE_NOT_RECOGNIZED), X(WRITE), X(READ)

    #define X(error) error
    enum class Error { SUSPENSION_ERRORS_DEF(X) };
    #undef X

    #define max_speed 50.0f
    #define max_acceleration_per_sec 100.0f

    static float target_left_speed = 0.0f;
    static float target_right_speed = 0.0f;
    
    static float previous_left_speed = 0.0f;
    static float previous_right_speed = 0.0f;
    
    static float previous_time = 0.0f;

    const char* get_error_string(Error e);

    static int init() {
        if (can_init() == 0) { return 0; }
        else { return 1; }
    };

    static bool target_speed_achieved() {
        return (target_left_speed == previous_left_speed && target_right_speed == previous_right_speed);
    };

    static int update_drive(float left_speed, float right_speed) {
        return can_send_drive(left_speed, right_speed, left_speed, right_speed, left_speed, right_speed);
    };

    static int update(int new_time) {
        //Time unchanged
        if (previous_time == new_time) { return 0; }

        //Get change in time
        float delta_time = (float(new_time) - previous_time) / 1000.0f;
        previous_time = float(new_time);

        //Nothing to update if target speed already reached
        if (target_speed_achieved()) { return 0; }

        //Find the actual speed of the motors
        float actual_left_speed = 0.0f;
        float actual_right_speed = 0.0f;

        //Set the left speed
        if (target_left_speed > actual_left_speed) {
            //Calculate the next speed
            actual_left_speed = previous_left_speed + (max_acceleration_per_sec * delta_time);

            //Cap the speed at the target
            if (target_left_speed < actual_left_speed) {
                actual_left_speed = target_left_speed;
            }
        }
        else if (target_left_speed < actual_left_speed) {
            //Calculate the next speed
            actual_left_speed = previous_left_speed - (max_acceleration_per_sec * delta_time);

            //Cap the speed at the target
            if (target_left_speed > actual_left_speed) {
                actual_left_speed = target_left_speed;
            }
        }
        else {
            //Actual is already the target
            actual_left_speed = target_left_speed;
        }
        //Save the left speed
        previous_left_speed = actual_left_speed;

        //Set the right speed
        if (target_right_speed > actual_right_speed) {
            //Calculate the next speed
            actual_right_speed = previous_right_speed + (max_acceleration_per_sec * delta_time);

            //Cap the speed at the target
            if (target_right_speed < actual_right_speed) {
                actual_right_speed = target_right_speed;
            }
        }
        else if (target_right_speed < actual_right_speed) {
            //Calculate the next speed
            actual_right_speed = previous_right_speed - (max_acceleration_per_sec * delta_time);

            //Cap the speed at the target
            if (target_right_speed > actual_right_speed) {
                actual_right_speed = target_right_speed;
            }
        }
        else {
            //Actual is already the target
            actual_right_speed = target_right_speed;
        }
        //Save the right speed
        previous_right_speed = actual_right_speed;

        //Send the speeds to the motors
        int ret = update_drive(actual_left_speed, actual_right_speed);

        //Return error value from CAN (currently 0 - success or 1 - failed write somewhere)
        return ret;
    };

    static int set_target_speed(int16_t in_left_speed, int16_t in_right_speed, int new_time) {
        //Set target speeds
        target_left_speed = (float(in_left_speed) / 255.0f) * max_speed;
        target_right_speed = (float(in_right_speed) / 255.0f) * max_speed;

        //Return error value from CAN (currently 0 - success or 1 - failed write somewhere)
        return update(new_time);
    };
} // namespace suspension

#endif