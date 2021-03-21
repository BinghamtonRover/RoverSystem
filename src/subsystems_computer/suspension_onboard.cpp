#include <cstring>
//#include <stdio.h>

#include "../rocs/rocs.hpp"

#include "suspension.hpp"

namespace suspension {

// Scale the input down by 2.
const uint16_t PRESCALE = 1;

uint8_t global_slave_addr;

// Assumes rocs has already been initialized!
Error init(uint8_t slave_addr) {
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
}

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


Error read_variable_velocity(){
    auto rocs_res = rocs::write_to_register(global_slave_addr, )
}

Error update(Side side, Direction direction, uint8_t speed) {
    auto rocs_res = rocs::write_to_register(global_slave_addr, side_to_speed_register_map[side], speed);
    if (rocs_res != rocs::Error::OK){
        return Error::WRITE;
    }

    rocs_res = rocs::write_to_register(global_slave_addr, side_to_dir_register_map[side], dir_to_value_map[direction]);
    if (rocs_res != rocs::Error::OK){
        return Error::WRITE;
    }    
    return Error::OK;
}

Error stop(Side side) {
    auto rocs_res = rocs::write_to_register(global_slave_addr, side_to_speed_register_map[side], 0);
    if (rocs_res != rocs::Error::OK){
        return Error::WRITE;
    }
    return Error::OK;
}
}
