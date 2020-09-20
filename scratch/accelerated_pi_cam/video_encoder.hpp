#ifndef VIDEO_ENCODER_H
#define VIDEO_ENCODER_H

#include "omx_video_component.hpp"
#include "omx_pi_camera.hpp"
#include "omx_encoder.hpp"
#include "camera_settings.hpp"

class VideoEncoder {
private:
	OMXPiCamera camera;
	OMXEncoder encoder;
	OMXVideoComponent null_sink;
	
public:
	VideoEncoder();
	//~VideoEncoder();
	
	// TODO: Options for multiplexed cameras, picture settings
	void init();
	void start_video();
	OMX_BUFFERHEADERTYPE* get_frame();
};

#endif
