#include <cstring>
//#include <stdio.h>

#include "../rocs/rocs.hpp"
#include "../can_bus/can_controller.hpp"

#include "suspension.hpp"

namespace suspension {

    // Scale the input down by 2.
    //this is never used, so I'm going to remove it - JM
    //const uint16_t PRESCALE = 1;

    //this is used in I2C so we don't need it anymore
    //uint8_t global_slave_addr;

    // Assumes rocs has already been initialized!
    //int init() {
    //There are no slave_addr, and the CAN device numbers should not change but I'll talk to EE about it - JM
    Error init(uint8_t slave_addr) {
        if (can_init_drive() == 0) { return Error::OK; }
        //I chose a random error here - JM
        else { return Error::READ; }
        /*
        global_slave_addr = slave_addr;

        // TODO: Prevent buffer overflows by taking buffer len in ROCS!
        char name_buffer[100];
        if (rocs::read_name(global_slave_addr, name_buffer) != rocs::Error::OK) {
            return Error::READ;
        }

        if (strcmp(name_buffer, "suspension") != 0) {
            return Error::DEVICE_NOT_RECOGNIZED;
        }

        return Error::OK;
        */
    }

    //More I2C stuff that we don't need anymore - JM
    /*
    uint8_t side_to_dir_register_map[2] = {
        [static_cast<int>(Side::LEFT)] = 0x05,
        [static_cast<int>(Side::RIGHT)] = 0x07,
    };

    uint8_t side_to_speed_register_map[2] = {
        [static_cast<int>(Side::LEFT)] = 0x06,
        [static_cast<int>(Side::RIGHT)] = 0x08,
    };

    uint8_t dir_to_value_map[2] = {
        [static_cast<int>(Direction::FORWARD)] = 0x00,
        [static_cast<int>(Direction::BACKWARD)] = 0x01,
    };

    const uint8_t READ_LEFT_VELOCITY = 0x09;
    const uint8_t READ_RIGHT_VELOCITY = 0x0A;
    */

    //not yet implemented with CAN (possible if necessary, would need to work with EE team to send messages from other devices) - JM
    //does not even compile - JM
    /*
    Error read_variable_velocity(){
        auto rocs_res = rocs::write_to_register(global_slave_addr, )
    }
    */

    //int update(Side side, Direction direction, uint8_t speed) {
    //new info is (wheel #, uint8_t speed) - JM
    Error update(Side side, Direction direction, uint8_t speed) {
        float float_speed = max_speed * (float)(speed / (uint8_t)255);
        
        /*
        if (direction == forward) {
            if (side == left) {
                //three of these should be min_speed (left ones)
                can_send_drive(float_speed, float_speed, float_speed, float_speed, float_speed, float_speed);
            }
            else if (side == right) {
                //three of these should be min_speed (right ones)
                can_send_drive(float_speed, float_speed, float_speed, float_speed, float_speed, float_speed);
            }
            else {
                can_send_drive(float_speed, float_speed, float_speed, float_speed, float_speed, float_speed);
            }
        }
        else if (direction == backward) {
            if (side == left) {

            }
            else if (side == right) {

            }
            else {
                
            }
        }
        else (direction == neutral?) {
            if (side == left) {

            }
            else if (side == right) {

            }
            else {
                //this should not be possible
            }
        }
        */


        //can_send(int device_num, float speed);
        /*
        auto rocs_res = rocs::write_to_register(global_slave_addr, side_to_speed_register_map[side], speed);
        if (rocs_res != rocs::Error::OK){
            return Error::WRITE;
        }

        rocs_res = rocs::write_to_register(global_slave_addr, side_to_dir_register_map[side], dir_to_value_map[direction]);
        if (rocs_res != rocs::Error::OK){
            return Error::WRITE;
        }    
        return Error::OK;
        */
    }

    //searched the drive and couldn't find anything, I'm assuming this was meant to shut down I2C - JM
    //when we leave the sockets open for can, this is where they will be closed, for now it's not needed for anything - JM
    Error stop(Side side) {
        /*
        auto rocs_res = rocs::write_to_register(global_slave_addr, side_to_speed_register_map[side], 0);
        if (rocs_res != rocs::Error::OK){
            return Error::WRITE;
        }
        return Error::OK;
        */
    }
}
