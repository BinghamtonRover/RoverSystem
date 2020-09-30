#ifndef OMX_ENCODER_H
#define OMX_ENCODER_H

#include "omx_video_component.hpp"
#include "camera_settings.hpp"
#include "h264_settings.hpp"

class OMXEncoder : public OMXVideoComponent {
private:
	OMX_BUFFERHEADERTYPE* output_buffer;
public:
	void set_port_definitions(const CameraSettings& settings);
	void set_h264(const H264Settings& settings);
	void enable_output_port();
	void disable_output_port();
	void fill_output_buffer();
	OMX_BUFFERHEADERTYPE* get_output_buffer();
};

#endif
