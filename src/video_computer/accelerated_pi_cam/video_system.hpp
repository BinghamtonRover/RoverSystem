#ifndef VIDEO_SYSTEM_H
#define VIDEO_SYSTEM_H

#include "../../network/network.hpp"
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
	enum {
		STOPPED, FILLING, FINISHED
	} async_buffer_status;
	
public:
	VideoSystem();
	
	// Initialize the camera and encoder with default settings.
	void init();
	void deinit();

	// Initialize the camera and encoder with provided settings. All values are copied and can be local variables.
	void init(CameraSettings& cam_settings, H264Settings& encoder_settings);
	
	// Begin capturing and encoding frames. Must init() first or there will be errors
	void start_video();
	void stop_video();
	OMX_BUFFERHEADERTYPE* get_partial_frame();

	/** Retrieve part of a full frame (enough to fill an OMX buffer or until the frame ends)
	 * 
	 *	@param buf Pointer that will be changed to point to the filled buffer
	 *  @returns true if this buffer starts a new frame. false if it continues an existing frame
	*/
	bool get_partial_frame(OMX_BUFFERHEADERTYPE** buf);
	
	/** begin filling the partial frame buffer
	 *  returns immediately
	 */
	void fill_partial_frame_async();
	
	// returns true if the partial frame buffer is filled
	bool partial_frame_available_async();
	
	/** same as get_partial_frame, but fill_partial_frame_async() should be called first
	 *  Blocking: if buffer not filled, this will wait for it to fill
	 *  Use partial_frame_available_async() to check before calling
	 * 
	 *  @param buf Pointer that will be changed to point to the filled buffer
	 *  @returns true if this buffer starts a new frame. false if it continues an existing frame
	 */
	bool get_partial_frame_async(OMX_BUFFERHEADERTYPE** buf);
	
	
	/** Utility class for splitting frames into messages for the network library */
	class MessageBuilder {
	public:
		/** Prepares to start a new frame.
		If delimiter is necessary, returns true and sets up msg as a delimiter */
		bool get_frame_delimiter(network::CSICameraMessage& msg, uint8_t stream=0);

		/** Preparing to break a partial frame into messages */
		void start_partial_frame(OMX_BUFFERHEADERTYPE* fptr);

		inline size_t available() {
			return frame_ptr->nFilledLen - (current_ptr - frame_ptr->pBuffer);
		}
		void fill_message(network::CSICameraMessage& msg);
	private:
		OMX_BUFFERHEADERTYPE* frame_ptr;
		uint16_t frame_index = 0;
		uint8_t section_index = 0;
		uint8_t stream_index = 0;
		uint8_t* current_ptr;
		bool beginning_of_stream = true;
	};
};

#endif
