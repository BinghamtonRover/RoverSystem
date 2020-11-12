#ifndef VIDEO_SYSTEM_H
#define VIDEO_SYSTEM_H

#include <network.hpp>
#include <iostream>
#include "omx_video_component.hpp"
#include "omx_pi_camera.hpp"
#include "omx_encoder.hpp"
#include "camera_settings.hpp"
#include "h264_settings.hpp"


class VideoSystem {
private:
	OMXPiCamera camera;
	OMXEncoder encoder;
	OMXVideoComponent null_sink;
	
public:
	VideoSystem();
	
	void init();
	void deinit();
	void init(CameraSettings& cam_settings, H264Settings& encoder_settings);
	void start_video();
	void stop_video();
	OMX_BUFFERHEADERTYPE* get_partial_frame();
	bool get_partial_frame(OMX_BUFFERHEADERTYPE** buf);	// Returns true if its a new frame, null buf is undefined behavior
	
	class MessageBuilder {
	public:
		bool start_new_frame(network::CSICameraMessage& msg, uint8_t stream=0);
		void start_partial_frame(OMX_BUFFERHEADERTYPE* fptr);
		inline bool available() {
			return frame_ptr->pBuffer + frame_ptr->nFilledLen > current_ptr;
		}
		void fill_message(network::CSICameraMessage& msg);
	private:
		OMX_BUFFERHEADERTYPE* frame_ptr;
		uint16_t frame_index = 0;
		uint8_t section_index;
		uint8_t stream_index;
		uint8_t* current_ptr;
		bool beginning_of_stream = true;
	};
};

#endif
