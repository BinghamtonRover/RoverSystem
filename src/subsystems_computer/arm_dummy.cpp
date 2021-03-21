#include "../network/network.hpp"
#include "../logger/logger.hpp"

#include "arm.hpp"

namespace arm {

Error init(uint8_t slave_addr) {
    return Error::OK;
}

Error update(network::ArmMessage::Joint joint, network::ArmMessage::Movement movement) {
    return Error::OK;
}

}
