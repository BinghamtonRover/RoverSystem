#include "imu.hpp"

namespace imu {

Error start(const char* device_serial_id) {
    return Error::OK;
}

Rotation get_rotation() {
    return { 0, 0, 0 };
}

Velocity get_velocity() {
    return {30, 25, 0.554};
}

float get_heading() {
    return -40.0f;
}

}
