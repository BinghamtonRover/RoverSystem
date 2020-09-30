#include "omx_encoder.hpp"

#include <IL/OMX_Broadcom.h>

#include "video_system_exception.hpp"

#define OMX_INIT_STRUCTURE(x) \
  memset (&(x), 0, sizeof (x)); \
  (x).nSize = sizeof (x); \
  (x).nVersion.nVersion = OMX_VERSION; \
  (x).nVersion.s.nVersionMajor = OMX_VERSION_MAJOR; \
  (x).nVersion.s.nVersionMinor = OMX_VERSION_MINOR; \
  (x).nVersion.s.nRevision = OMX_VERSION_REVISION; \
  (x).nVersion.s.nStep = OMX_VERSION_STEP

void OMXEncoder::set_port_definitions(const CameraSettings& settings) {
	OMX_PARAM_PORTDEFINITIONTYPE port_st;
	OMX_INIT_STRUCTURE(port_st);
	port_st.nPortIndex = 201;
	VideoSystemException::omx_error_check("encoder", "get port definition", OMX_GetParameter(handle, OMX_IndexParamPortDefinition, &port_st));
	
	port_st.format.video.nFrameWidth = settings.width;
	port_st.format.video.nFrameHeight = settings.height;
	port_st.format.video.nStride = settings.width;
	port_st.format.video.xFramerate = settings.framerate << 16;
	port_st.format.video.nBitrate = settings.bitrate;
	VideoSystemException::omx_error_check("encoder", "set port definition", OMX_SetParameter(handle, OMX_IndexParamPortDefinition, &port_st));
	
}

void OMXEncoder::set_h264(const H264Settings& settings) {
	OMX_PARAM_PORTDEFINITIONTYPE port_st;
	VideoSystemException::omx_error_check("encoder", "h264 get port definition", OMX_GetParameter(handle, OMX_IndexParamPortDefinition, &port_st));
	
	if (!settings.qp) {
		OMX_VIDEO_PARAM_BITRATETYPE bitrate_st;
		OMX_INIT_STRUCTURE(bitrate_st);
		bitrate_st.eControlRate = OMX_Video_ControlRateVariable;
		bitrate_st.nTargetBitrate = port_st.format.video.nBitrate;
		bitrate_st.nPortIndex = 201;
		VideoSystemException::omx_error_check("encoder", "h264 bitrate", OMX_SetParameter(handle, OMX_IndexParamVideoBitrate, &bitrate_st));
	} else {
		port_st.format.video.nBitrate = 0;	// Must be 0 if quantization is enabled
		VideoSystemException::omx_error_check("encoder", "h264 disable bitrate", OMX_SetParameter(handle, OMX_IndexParamPortDefinition, &port_st));
		OMX_VIDEO_PARAM_QUANTIZATIONTYPE quantization_st;
		OMX_INIT_STRUCTURE(quantization_st);
		quantization_st.nPortIndex = 201;
		quantization_st.nQpI = settings.qp_i;
		quantization_st.nQpP = settings.qp_p;
		VideoSystemException::omx_error_check("encoder", "h264 set quantization", OMX_SetParameter(handle, OMX_IndexParamVideoQuantization, &quantization_st));
	}
	
	OMX_VIDEO_PARAM_PORTFORMATTYPE format_st;
	OMX_INIT_STRUCTURE(format_st);
	format_st.nPortIndex = 201;
	format_st.eCompressionFormat = OMX_VIDEO_CodingAVC;
	VideoSystemException::omx_error_check("encoder", "h264 set format", OMX_SetParameter(handle, OMX_IndexParamVideoPortFormat, &format_st));
	
	OMX_VIDEO_CONFIG_AVCINTRAPERIOD idr_st;
	OMX_INIT_STRUCTURE (idr_st);
	idr_st.nPortIndex = 201;
	VideoSystemException::omx_error_check("encoder", "h264 get IDR", OMX_GetConfig(handle, OMX_IndexConfigVideoAVCIntraPeriod, &idr_st));
	idr_st.nIDRPeriod = settings.idr_period;
	VideoSystemException::omx_error_check("encoder", "h264 set IDR", OMX_SetConfig(handle, OMX_IndexConfigVideoAVCIntraPeriod, &idr_st));
	
	OMX_PARAM_BRCMVIDEOAVCSEIENABLETYPE sei_st;
	OMX_INIT_STRUCTURE (sei_st);
	sei_st.nPortIndex = 201;
	sei_st.bEnable = settings.sei;
	VideoSystemException::omx_error_check("encoder", "h264 set sei", OMX_SetParameter(handle, OMX_IndexParamBrcmVideoAVCSEIEnable, &sei_st));

	OMX_VIDEO_EEDE_ENABLE eede_st;
	OMX_INIT_STRUCTURE (eede_st);
	eede_st.nPortIndex = 201;
	eede_st.enable = settings.eede;
	VideoSystemException::omx_error_check("encoder", "h264 set eede", OMX_SetParameter(handle, OMX_IndexParamBrcmEEDEEnable, &eede_st));

	OMX_VIDEO_EEDE_LOSSRATE eede_loss_rate_st;
	OMX_INIT_STRUCTURE (eede_loss_rate_st);
	eede_loss_rate_st.nPortIndex = 201;
	eede_loss_rate_st.loss_rate = settings.eede_loss_rate;
	VideoSystemException::omx_error_check("encoder", "h264 set eede loss rate", OMX_SetParameter(handle, OMX_IndexParamBrcmEEDELossRate, &eede_loss_rate_st));

	OMX_VIDEO_PARAM_AVCTYPE avc_st;
	OMX_INIT_STRUCTURE (avc_st);
	avc_st.nPortIndex = 201;
	VideoSystemException::omx_error_check("encoder", "h264 get avc", OMX_GetParameter(handle, OMX_IndexParamVideoAvc, &avc_st));
	
	avc_st.eProfile = OMX_VIDEO_AVCProfileHigh;
	VideoSystemException::omx_error_check("encoder", "h264 set avc", OMX_SetParameter(handle, OMX_IndexParamVideoAvc, &avc_st));

	OMX_CONFIG_PORTBOOLEANTYPE headers_st;
	OMX_INIT_STRUCTURE (headers_st);
	headers_st.nPortIndex = 201;
	headers_st.bEnabled = settings.inline_headers;
	VideoSystemException::omx_error_check("encoder", "h264 set inline", OMX_SetParameter(handle, OMX_IndexParamBrcmVideoAVCInlineHeaderEnable, &headers_st));
	
}

void OMXEncoder::enable_output_port() {
	enable_port(201);
	OMX_PARAM_PORTDEFINITIONTYPE port_st;
	OMX_INIT_STRUCTURE(port_st);
	port_st.nPortIndex = 201;
	VideoSystemException::omx_error_check("encoder", "get buffer", OMX_GetParameter(handle, OMX_IndexParamPortDefinition, &port_st));
	VideoSystemException::omx_error_check("encoder", "allocate buffer", OMX_AllocateBuffer(handle, &output_buffer, 201, 0, port_st.nBufferSize));
	wait_event(ComponentEvent::PORT_ENABLE);
	
}

void OMXEncoder::disable_output_port() {
	disable_port(201);
	VideoSystemException::omx_error_check("encoder", "release output buffer", OMX_FreeBuffer(handle, 201, output_buffer));
	wait_event(ComponentEvent::PORT_DISABLE);
}

void OMXEncoder::fill_output_buffer() {
	VideoSystemException::omx_error_check("encoder", "fill output buffer", OMX_FillThisBuffer(handle, output_buffer));
}

OMX_BUFFERHEADERTYPE* OMXEncoder::get_output_buffer() {
	return output_buffer;
}
