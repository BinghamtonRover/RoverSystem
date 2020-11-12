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

OMX_BUFFERHEADERTYPE* VideoSystem::get_partial_frame() {
	encoder.fill_output_buffer();
	encoder.wait_event(ComponentEvent::FILL_BUFFER_DONE);
	return encoder.get_output_buffer();
}

bool VideoSystem::get_partial_frame(OMX_BUFFERHEADERTYPE** buf) {
	encoder.fill_output_buffer();
	encoder.wait_event(ComponentEvent::FILL_BUFFER_DONE);
	OMX_BUFFERHEADERTYPE* outbuf = encoder.get_output_buffer();
	*buf = outbuf;
	// H264 units begin with 0x00000001, comparison backwards for endianness
	return outbuf->nFilledLen >= 4 && *((uint32_t*)(outbuf->pBuffer)) == 0x01000000;
}

bool VideoSystem::MessageBuilder::start_new_frame(network::CSICameraMessage& msg, uint8_t stream) {
	msg.size = 0;
	msg.stream_index = stream_index;
	msg.frame_index = frame_index;
	msg.section_index = 0;
	if (!beginning_of_stream) {
		// A message with no data will indicates its the last section
		frame_index += 1;
		stream_index = stream;
		return true;
	} else {
		beginning_of_stream = false;
		return false;
	}

}

void VideoSystem::MessageBuilder::start_partial_frame(OMX_BUFFERHEADERTYPE* fptr) {
	frame_ptr = fptr;
	current_ptr = fptr->pBuffer;
}

void VideoSystem::MessageBuilder::fill_message(network::CSICameraMessage& msg) {
	size_t remaining = frame_ptr->nFilledLen - (current_ptr - frame_ptr->pBuffer);
	if (remaining > network::CSICameraMessage::MAX_DATA)
		msg.size = network::CSICameraMessage::MAX_DATA;
	else
		msg.size = remaining;
	msg.stream_index = stream_index;
	msg.frame_index = frame_index;
	msg.section_index = section_index;
	msg.data = current_ptr;
	section_index += 1;
	
	current_ptr += msg.size;
	
}
