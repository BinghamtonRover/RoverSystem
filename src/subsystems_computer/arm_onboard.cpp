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

    if (strcmp(name_buffer, "arm") != 0) {
        return Error::DEVICE_NOT_RECOGNIZED;
    }

    return Error::OK;
}

uint8_t joint_to_reg_map[6] = {
    [static_cast<int>(network::ArmMessage::Joint::BASE_ROTATE)] = 0x01,
    [static_cast<int>(network::ArmMessage::Joint::BASE_SHOULDER)] = 0x02,
    [static_cast<int>(network::ArmMessage::Joint::ELBOW)] = 0x03,
    [static_cast<int>(network::ArmMessage::Joint::WRIST)] = 0x04,
    [static_cast<int>(network::ArmMessage::Joint::GRIPPER_ROTATE)] = 0x05,
    [static_cast<int>(network::ArmMessage::Joint::GRIPPER_FINGERS)] = 0x06,
};

uint8_t state_to_value_map[3] = {
    [static_cast<int>(network::ArmMessage::Movement::STOP)] = 0x02,
    [static_cast<int>(network::ArmMessage::Movement::CLOCK)] = 0x00,
    [static_cast<int>(network::ArmMessage::Movement::COUNTER)] = 0x01,
};

Error update(network::ArmMessage::Joint joint, network::ArmMessage::Movement movement) {
    auto rocs_res = rocs::write_to_register(global_slave_addr, joint_to_reg_map[static_cast<int>(joint)], state_to_value_map[static_cast<int>(movement)]);
    if (rocs_res != rocs::Error::OK) return Error::WRITE;

    return Error::OK;
}

}
