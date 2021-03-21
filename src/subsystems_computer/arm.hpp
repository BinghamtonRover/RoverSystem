#include "../network/network.hpp"

namespace arm {

enum class Error {
    OK,
    DEVICE_NOT_RECOGNIZED,
    READ,
    WRITE
};

Error init(uint8_t slave_addr);

Error update(network::ArmMessage::Joint motor, network::ArmMessage::Movement state);

}
