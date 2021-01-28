#include "suspension.hpp"

#include <cstdio>

namespace suspension {

Error init(uint8_t slave_addr) {
    return Error::OK;
}

Error update(Side side, Direction direction, uint8_t speed) {
    // printf("> Updating side %d, Direction %d, with speed %u\n", (int) side, (int) direction, speed);
    return Error::OK;
}

Error stop(Side side) {
    return Error::OK;
}

} // namespace suspension
