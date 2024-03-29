#include "video_system.hpp"

#include <bcm_host.h>
#include <IL/OMX_Broadcom.h>
#include <iostream>

#include "component_event.hpp"
#include "video_system_exception.hpp"

VideoSystem::VideoSystem() {
	camera.name = (char*) "OMX.broadcom.camera";
	encoder.name = (char*) "OMX.broadcom.video_encode";
	null_sink.name = (char*) "OMX.broadcom.null_sink";
}

void VideoSystem::init() {
	CameraSettings cam_settings_default;
	H264Settings encoder_settings_default;
	init(cam_settings_default, encoder_settings_default);
}

void VideoSystem::init(CameraSettings& cam_settings, H264Settings& encoder_settings) {
	bcm_host_init();
	VideoSystemException::omx_error_check("OMX", "init", OMX_Init());
	
	camera.init();
	encoder.init();
	null_sink.init();
	
	camera.load_drivers();
	camera.set_port_definitions(cam_settings);
	camera.apply_settings(cam_settings);
	
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

void VideoSystem::start_video() {
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

void VideoSystem::stop_video() {
	camera.change_state(OMX_StateIdle);
	camera.wait_event(ComponentEvent::STATE_SET);
	encoder.change_state(OMX_StateIdle);
	encoder.wait_event(ComponentEvent::STATE_SET);
	null_sink.change_state(OMX_StateIdle);
	null_sink.wait_event(ComponentEvent::STATE_SET);
	
	camera.disable_port(71);
	camera.wait_event(ComponentEvent::PORT_DISABLE);
	camera.disable_port(70);
	camera.wait_event(ComponentEvent::PORT_DISABLE);
	null_sink.disable_port(240);
	null_sink.wait_event(ComponentEvent::PORT_DISABLE);
	encoder.disable_port(200);
	encoder.wait_event(ComponentEvent::PORT_DISABLE);
	encoder.disable_output_port();
	
	camera.change_state(OMX_StateLoaded);
	camera.wait_event(ComponentEvent::STATE_SET);
	encoder.change_state(OMX_StateLoaded);
	encoder.wait_event(ComponentEvent::STATE_SET);
	null_sink.change_state(OMX_StateLoaded);
	null_sink.wait_event(ComponentEvent::STATE_SET);
}

void VideoSystem::deinit() {
	camera.deinit();
	encoder.deinit();
	null_sink.deinit();
	
	VideoSystemException::omx_error_check("video system", "deinit", OMX_Deinit());
	
	bcm_host_deinit();
}

OMX_BUFFERHEADERTYPE* VideoSystem::get_frame() {
	encoder.fill_output_buffer();
	encoder.wait_event(ComponentEvent::FILL_BUFFER_DONE);
	return encoder.get_output_buffer();
}
