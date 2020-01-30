#include "suspension.hpp"

#include <cstdio>

namespace suspension {

Error init(const char* i2c_dev_path, uint8_t slave_addr) {
    return Error::OK;
}

Error update(Side side, Direction direction, uint8_t speed) {
    // printf("> Updating side %d, Direction %d, with speed %u\n", (int) side, (int) direction, speed);
    return Error::OK;
}

} // namespace suspension
