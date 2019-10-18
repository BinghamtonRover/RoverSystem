#include "../util/util.hpp"

namespace zed {

enum class Error {
    OK,

    OPEN,
    TRACKING,
    GRAB
};

Error open(util::Clock* clock);

struct Pose {
    float x, y, z, pitch, yaw, roll;
};

Error grab(unsigned char** out_frame, int* out_stride, Pose* out_pose);

} // namespace zed
