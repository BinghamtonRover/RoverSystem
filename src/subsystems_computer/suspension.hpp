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

    #define delta_time_cap 0.16f
    #define max_no_message_time 5.0f

    static float target_left_speed = 0.0f;
    static float target_right_speed = 0.0f;
    
    static float previous_left_speed = 0.0f;
    static float previous_right_speed = 0.0f;
    
    static int previous_time = 0;
    static float no_message_time = 0.0f;

    const char* get_error_string(Error e);

    //The first part of initializing Odrives, where they calibrate themselves
    //This takes a few seconds (not consistent timing as well), and blocks other CAN messages
    static int init_calibrate() {
        int ret = 0;

        char drive_0_axis_0_calibration[] = "007#03000000";
	    ret += can_send_custom_message(drive_0_axis_0_calibration);
        char drive_0_axis_1_calibration[] = "027#03000000";
	    ret += can_send_custom_message(drive_0_axis_1_calibration);
        char drive_1_axis_0_calibration[] = "047#03000000";
	    ret += can_send_custom_message(drive_1_axis_0_calibration);
        char drive_1_axis_1_calibration[] = "067#03000000";
        ret += can_send_custom_message(drive_1_axis_1_calibration);
        char drive_2_axis_0_calibration[] = "087#03000000";
        ret += can_send_custom_message(drive_2_axis_0_calibration);
        char drive_2_axis_1_calibration[] = "0a7#03000000";
	    ret += can_send_custom_message(drive_2_axis_1_calibration);
        
        //If all the can sends return 0, there are no write errors
        if (ret == 0) { return 0; }
        return 1;
    };

    //The second part of initializing Odrives, send the proper settings
    static int init_setup_control() {
        int ret = 0;

        char drive_0_axis_0_loop_control[] = "007#08000000";
        ret += can_send_custom_message(drive_0_axis_0_loop_control);
        char drive_0_axis_1_loop_control[] = "027#08000000";
        ret += can_send_custom_message(drive_0_axis_1_loop_control);
        char drive_1_axis_0_loop_control[] = "047#08000000";
        ret += can_send_custom_message(drive_1_axis_0_loop_control);
        char drive_1_axis_1_loop_control[] = "067#08000000";
        ret += can_send_custom_message(drive_1_axis_1_loop_control);
        char drive_2_axis_0_loop_control[] = "087#08000000";
        ret += can_send_custom_message(drive_2_axis_0_loop_control);
        char drive_2_axis_1_loop_control[] = "0a7#08000000";
        ret += can_send_custom_message(drive_2_axis_1_loop_control);

        char drive_0_axis_0_velocity_control[] = "40b#02000000";
        ret += can_send_custom_message(drive_0_axis_0_velocity_control);
        char drive_0_axis_1_velocity_control[] = "42b#02000000";
        ret += can_send_custom_message(drive_0_axis_1_velocity_control);
        char drive_1_axis_0_velocity_control[] = "44b#02000000";
        ret += can_send_custom_message(drive_1_axis_0_velocity_control);
        char drive_1_axis_1_velocity_control[] = "46b#02000000";
        ret += can_send_custom_message(drive_1_axis_1_velocity_control);
        char drive_2_axis_0_velocity_control[] = "48b#02000000";
        ret += can_send_custom_message(drive_2_axis_0_velocity_control);
        char drive_2_axis_1_velocity_control[] = "4ab#02000000";
        ret += can_send_custom_message(drive_2_axis_1_velocity_control);

        //If all the can sends return 0, there are no write errors
        if (ret == 0) { return 0; }
        return 1;
    }

    static bool target_speed_achieved() {
        return (target_left_speed == previous_left_speed && target_right_speed == previous_right_speed);
    };

    static int update_drive(float left_speed, float right_speed) {
        int ret = 0;

        //Send velocity to each motor
	    ret += can_send_velocity(0, left_speed);
	    ret += can_send_velocity(1, right_speed);
	    ret += can_send_velocity(2, left_speed);
	    ret += can_send_velocity(3, right_speed);
	    ret += can_send_velocity(4, left_speed);
	    ret += can_send_velocity(5, right_speed);

        //If all the can sends return 0, there are no write errors
	    if (ret == 0) { return 0; }
	    else { return 1; }
    };

    static int update(int new_time) {
        //Time unchanged
        if (previous_time == new_time) { return 0; }

        //Get change in time
        float delta_time = (float(new_time - previous_time)) / 1000.0f;
        previous_time = new_time;
        no_message_time += delta_time;

        //Zero the speed if no messages sent for too long
        if (no_message_time > max_no_message_time) {
            target_left_speed = 0;
            target_right_speed = 0;
        }

        //Cap the delta time
        if (delta_time > delta_time_cap) { delta_time = delta_time_cap; }

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

        //Update the time since the last message has been sent
        no_message_time = 0;

        //Return error value from CAN (currently 0 - success or 1 - failed write somewhere)
        return update(new_time);
    };
} // namespace suspension

#endif