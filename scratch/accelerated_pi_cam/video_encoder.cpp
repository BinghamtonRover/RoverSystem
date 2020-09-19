#include "video_encoder.hpp"

#include <bcm_host.h>
#include <IL/OMX_Broadcom.h>
#include <iostream>

#include "component_event.hpp"

VideoEncoder::VideoEncoder() {
	camera.name = (char*) "OMX.broadcom.camera";
	encoder.name = (char*) "OMX.broadcom.video_encode";
	null_sink.name = (char*) "OMX.broadcom.null_sink";
}

void VideoEncoder::init_camera_system() {
	bcm_host_init();
	
	OMX_ERRORTYPE error = OMX_Init();
	
	if (error) {
		std::cout << "Error initializing OMX: " << OMXVideoComponent::get_error_name(error) << std::endl;
		return;
	}
	
	if (!camera.init()) {
		std::cout << "Error initializing component: camera." << std::endl;
		return;
	}
		
	if (!encoder.init()) {
		std::cout << "Error initializing component: encoder." << std::endl;
		return;
	}
	
	if (!null_sink.init()) {
		std::cout << "Error initializing component: null_sink." << std::endl;
		return;
	}
	
	camera.load_drivers();
	
	
}
