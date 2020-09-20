#include "video_encoder.hpp"

#include <bcm_host.h>
#include <IL/OMX_Broadcom.h>
#include <iostream>

#include "component_event.hpp"
#include "h264_settings.hpp"
#include "video_system_exception.hpp"

VideoEncoder::VideoEncoder() {
	camera.name = (char*) "OMX.broadcom.camera";
	encoder.name = (char*) "OMX.broadcom.video_encode";
	null_sink.name = (char*) "OMX.broadcom.null_sink";
}

void VideoEncoder::init() {
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
	CameraSettings cam_settings;
	camera.set_port_definitions(cam_settings);
	camera.apply_settings(cam_settings);
	
	H264Settings encoder_settings;
	encoder.set_port_definitions(cam_settings);
	encoder.set_h264(encoder_settings);
	
	VideoSystemException::omx_error_check("system buffer tunnel", "camera->encoder", OMX_SetupTunnel(camera.handle, 71, encoder.handle, 200));
	
	VideoSystemException::omx_error_check("system buffer tunnel", "camera->null_sink", OMX_SetupTunnel(camera.handle, 70, null_sink.handle, 240));
	
	camera.change_state(OMX_StateIdle);
	camera.wait_event(ComponentEvent::STATE_SET);
	encoder.change_state(OMX_StateIdle);
	encoder.wait_event(ComponentEvent::STATE_SET);
	null_sink.change_state(OMX_StateIdle);
	null_sink.wait_event(ComponentEvent::STATE_SET);
	
}

void VideoEncoder::start_video() {
	camera.enable_port(71);
	camera.wait_event(ComponentEvent::PORT_ENABLE);
	camera.enable_port(70);
	camera.wait_event(ComponentEvent::PORT_ENABLE);
	null_sink.enable_port(240);
	null_sink.wait_event(ComponentEvent::PORT_ENABLE);
	encoder.enable_port(200);
	encoder.wait_event(ComponentEvent::PORT_ENABLE);
	encoder.enable_output_port();
	
	camera.change_state(OMX_StateExecuting);
	camera.wait_event(ComponentEvent::STATE_SET);
	encoder.change_state(OMX_StateExecuting);
	encoder.wait_event(ComponentEvent::STATE_SET);
	encoder.wait_event(ComponentEvent::PORT_SETTINGS_CHANGED);
	null_sink.change_state(OMX_StateExecuting);
	null_sink.wait_event(ComponentEvent::STATE_SET);
	
	camera.start_video_capture();
}

OMX_BUFFERHEADERTYPE* VideoEncoder::get_frame() {
	encoder.fill_output_buffer();
	encoder.wait_event(ComponentEvent::FILL_BUFFER_DONE);
	return encoder.get_output_buffer();
}
