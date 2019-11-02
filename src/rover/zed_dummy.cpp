#include "zed.hpp"
#include "subsystem.hpp"

#include <string.h>

namespace zed {

static const int FPS = 30;
static const int FRAME_UPDATE_INTERVAL = 1000 / FPS;

static util::Timer timer;

static uint8_t subsystem_id;

Error open(util::Clock* clock) {
    subsystem_id = subsystem::add("ZED Camera (dummy)");

    // For the dummy, just immediately set status to connected.
    subsystem::get(subsystem_id)->state = subsystem::State::CONNECTED;

    util::Timer::init(&timer, (uint32_t) FRAME_UPDATE_INTERVAL, clock);

    return Error::OK;
}

Error grab(unsigned char** out_frame, int* out_stride, Pose* out_pose) {
    if (!timer.ready()) {
        return Error::GRAB;
    }

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

} // namespace zed
