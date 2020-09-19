#ifndef VIDEO_ENCODER_H
#define VIDEO_ENCODER_H

#include "omx_video_component.hpp"
#include "omx_pi_camera.hpp"

class VideoEncoder {
private:
	OMXPiCamera camera;
	OMXVideoComponent encoder;
	OMXVideoComponent null_sink;
	
public:
	VideoEncoder();
	//~VideoEncoder();
	
	// TODO: Options for multiplexed cameras, picture settings
	void init_camera_system();
};

#endif
