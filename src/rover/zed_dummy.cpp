#include "zed.hpp"

#include <string.h>

namespace zed {

Error open() {
    return Error::OK;
}

Error grab(unsigned char** out_frame, int* out_stride, Pose* out_pose) {
    (*out_pose).x = 0;
    (*out_pose).y = 0;
    (*out_pose).z = 0;
    (*out_pose).pitch = 0;
    (*out_pose).yaw = 0;
    (*out_pose).roll = 0;

    *out_stride = 1280 * 3;

    static unsigned char image_buffer[1280 * 720 * 3];
    memset(image_buffer, 0, 1280 * 720 * 3);

    *out_frame = image_buffer;

    return Error::OK;
}

}
