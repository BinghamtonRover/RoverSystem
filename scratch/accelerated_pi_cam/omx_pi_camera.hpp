#ifndef OMX_PI_CAMERA
#define OMX_PI_CAMERA

#include "omx_video_component.hpp"
#include "camera_settings.hpp"

class OMXPiCamera : public OMXVideoComponent {
public:
	void load_drivers();
	void set_port_definitions(const CameraSettings& settings);
	void apply_settings(const CameraSettings& settings);
	void start_video_capture();
};

#endif
