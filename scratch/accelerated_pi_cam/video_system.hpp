#ifndef VIDEO_SYSTEM_H
#define VIDEO_SYSTEM_H

#include "omx_video_component.hpp"
#include "omx_pi_camera.hpp"
#include "omx_encoder.hpp"
#include "camera_settings.hpp"

class VideoSystem {
private:
	OMXPiCamera camera;
	OMXEncoder encoder;
	OMXVideoComponent null_sink;
	
public:
	VideoSystem();
	
	// TODO: Options for multiplexed cameras, picture settings
	void init();
	void start_video();
	OMX_BUFFERHEADERTYPE* get_frame();
};

#endif
