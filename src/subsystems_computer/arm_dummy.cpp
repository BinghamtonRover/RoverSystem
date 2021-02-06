#include "../network/network.hpp"

#include "arm.hpp"

namespace arm {

Error init(uint8_t slave_addr) {
    return Error::OK;
}

Error update(network::ArmMessage::Motor motor, network::ArmMessage::State state) {
    return Error::OK;
}

}
