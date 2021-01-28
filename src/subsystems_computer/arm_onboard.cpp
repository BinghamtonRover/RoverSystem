#include <cstring>

#include "../rocs/rocs.hpp"

#include "arm.hpp"

namespace arm {

uint8_t global_slave_addr;

Error init(uint8_t slave_addr) {
    global_slave_addr = slave_addr;

    // TODO: Prevent buffer overflows by taking buffer len in ROCS!
    char name_buffer[100];
    if (rocs::read_name(global_slave_addr, name_buffer) != rocs::Error::OK) {
        return Error::READ;
    }

    if (strcmp(name_buffer, "ptaArm") != 0) {
        return Error::DEVICE_NOT_RECOGNIZED;
    }

    return Error::OK;
}

static uint8_t get_register(network::ArmMessage::Motor motor) {
    switch (motor) {
        case network::ArmMessage::Motor::ARM_LOWER:
            return 0x01;
        case network::ArmMessage::Motor::ARM_UPPER:
            return 0x02;
        case network::ArmMessage::Motor::ARM_BASE:
            return 0x03;
        default:
            return 0x00;
    }
}

uint8_t state_to_value_map[3] = {
    [static_cast<int>(network::ArmMessage::State::STOP)] = 0x01,
    [static_cast<int>(network::ArmMessage::State::CLOCK)] = 0x00,
    [static_cast<int>(network::ArmMessage::State::COUNTER)] = 0x02,
};

Error update(network::ArmMessage::Motor motor, network::ArmMessage::State state) {
    auto rocs_res = rocs::write_to_register(
        global_slave_addr,
        get_register(motor),
        state_to_value_map[static_cast<int>(state)]);
    if (rocs_res != rocs::Error::OK) return Error::WRITE;

    return Error::OK;
}

}
