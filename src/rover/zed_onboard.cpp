#include "zed.hpp"

#include <sl/Camera.hpp>

namespace zed {

sl::Camera zed;

Error open() {
    // Open ZED camera.

    sl::InitParameters params;
    params.camera_resolution = sl::RESOLUTION_HD720;
    params.camera_fps = 30;
    params.coordinate_system = sl::COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP; // Use a right-handed Y-up coordinate system
    params.coordinate_units = sl::UNIT_METER; // Set units in meters

    auto zed_res = zed.open(params);
    if (zed_res != sl::SUCCESS) {
        return Error::OPEN;
    }

    sl::TrackingParameters tracking_parameters;
    zed_res = zed.enableTracking(tracking_parameters);
    if (zed_res != sl::SUCCESS) {
        return Error::TRACKING;
    }

    return Error::OK;
}

Error grab(unsigned char** out_frame, int* out_stride, Pose* out_pose) {
    static sl::Mat zed_image;

    if (zed.grab() != sl::SUCCESS) {
        return Error::GRAB;
    }

    zed.retrieveImage(zed_image, sl::VIEW_LEFT);

    *out_frame = zed_image.getPtr<unsigned char>();
    *out_stride = zed_image.getStepBytes();

    // ZED positional tracking.
    sl::Pose pose;
    sl::TRACKING_STATE state = zed.getPosition(pose, sl::REFERENCE_FRAME_WORLD);
    if (state != sl::TRACKING_STATE_OK) {
        return Error::TRACKING;
    }

    auto trans = pose.getTranslation();
    auto angles = pose.getEulerAngles(false);

    (*out_pose).x = trans.x;
    (*out_pose).y = trans.y;
    (*out_pose).z = trans.z;
    (*out_pose).pitch = angles[0];
    (*out_pose).yaw = angles[1];
    (*out_pose).roll = angles[2];

    return Error::OK;
}

} // namespace zed
