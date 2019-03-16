#include "imu.hpp"

namespace imu {

Error start(const char* device_serial_id) {
    return Error::OK;
}

Rotation get_rotation() {
    return { 0, 0, 0 };
}

}
