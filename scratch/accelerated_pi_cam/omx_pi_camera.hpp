#ifndef OMX_PI_CAMERA
#define OMX_PI_CAMERA

#include "omx_video_component.hpp"

class OMXPiCamera : public OMXVideoComponent {
public:
	void load_drivers();
	
};

#endif
