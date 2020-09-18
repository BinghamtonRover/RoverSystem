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

void VideoEncoder::load_camera_drivers() {
	OMX_ERRORTYPE error;
	
	OMX_CONFIG_REQUESTCALLBACKTYPE cbs;
	OMX_INIT_STRUCTURE(cbs);
	cbs.nPortIndex = OMX_ALL;
	cbs.nIndex = OMX_IndexParamCameraDeviceNumber;
	cbs.bEnable = OMX_TRUE;
	
	if (OMX_SetConfig(camera.handle, OMX_IndexConfigRequestCallback, &cbs)) {
		std::cout << "Error loading camera drivers (set config)" << std::endl;
		return;
	}
	
	OMX_PARAM_U32TYPE dev;
	OMX_INIT_STRUCTURE(dev);
	dev.nPortIndex = OMX_ALL;
	dev.nU32 = 0;
	if (OMX_SetParameter(camera.handle, OMX_IndexParamCameraDeviceNumber, &dev)) {
		std::cout << "Error loading camera drivers (set parameter)" << std::endl;
		return;
	}
	camera.wait_event(ComponentEvent::PARAM_OR_CONFIG_CHANGED, 0);
	
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
	
	load_camera_drivers();
	
	
}
