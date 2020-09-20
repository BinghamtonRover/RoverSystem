#include "omx_pi_camera.hpp"

#include <IL/OMX_Broadcom.h>
#include <iostream>

#include "video_system_exception.hpp"

#define OMX_INIT_STRUCTURE(x) \
  memset (&(x), 0, sizeof (x)); \
  (x).nSize = sizeof (x); \
  (x).nVersion.nVersion = OMX_VERSION; \
  (x).nVersion.s.nVersionMajor = OMX_VERSION_MAJOR; \
  (x).nVersion.s.nVersionMinor = OMX_VERSION_MINOR; \
  (x).nVersion.s.nRevision = OMX_VERSION_REVISION; \
  (x).nVersion.s.nStep = OMX_VERSION_STEP

void OMXPiCamera::load_drivers() {
	OMX_ERRORTYPE error;
	
	OMX_CONFIG_REQUESTCALLBACKTYPE cbs;
	OMX_INIT_STRUCTURE(cbs);
	cbs.nPortIndex = OMX_ALL;
	cbs.nIndex = OMX_IndexParamCameraDeviceNumber;
	cbs.bEnable = OMX_TRUE;
	
	if (OMX_SetConfig(handle, OMX_IndexConfigRequestCallback, &cbs)) {
		std::cout << "Error loading camera drivers (set config)" << std::endl;
		return;
	}
	
	OMX_PARAM_U32TYPE dev;
	OMX_INIT_STRUCTURE(dev);
	dev.nPortIndex = OMX_ALL;
	dev.nU32 = 0;
	if (OMX_SetParameter(handle, OMX_IndexParamCameraDeviceNumber, &dev)) {
		std::cout << "Error loading camera drivers (set parameter)" << std::endl;
		return;
	}
	wait_event(ComponentEvent::PARAM_OR_CONFIG_CHANGED, 0);
}

void OMXPiCamera::set_port_definitions(const CameraSettings& settings) {
	OMX_PARAM_PORTDEFINITIONTYPE port_st;
	OMX_INIT_STRUCTURE(port_st);
	port_st.nPortIndex = 71;
	OMX_ERRORTYPE error = OMX_GetParameter(handle, OMX_IndexParamPortDefinition, &port_st);
	if (error) {
		std::cout << "Error applying camera settings: (loading current settings)" << get_error_name(error) << std::endl;
		return;
	}
	
	port_st.format.video.nFrameWidth = settings.width;
	port_st.format.video.nFrameHeight = settings.height;
	port_st.format.video.nStride = settings.width;
	port_st.format.video.xFramerate = settings.framerate << 16;	// Q16 Format
	port_st.format.video.eCompressionFormat = OMX_VIDEO_CodingUnused;
	port_st.format.video.eColorFormat = OMX_COLOR_FormatYUV420PackedPlanar;
	
	error = OMX_SetParameter(handle, OMX_IndexParamPortDefinition, &port_st);
	if (error) {
		std::cout << "Error applying camera settings: (saving settings)" << get_error_name(error) << std::endl;
		return;
	}
	
	port_st.nPortIndex = 70;
	error = OMX_SetParameter(handle, OMX_IndexParamPortDefinition, &port_st);
	if (error) {
		std::cout << "Error applying camera settings: (saving settings preview)" << get_error_name(error) << std::endl;
		return;
	}
	
	OMX_CONFIG_FRAMERATETYPE framerate_st;
	OMX_INIT_STRUCTURE(framerate_st);
	framerate_st.nPortIndex = 71;
	framerate_st.xEncodeFramerate = port_st.format.video.xFramerate;
	error = OMX_SetConfig(handle, OMX_IndexConfigVideoFramerate, &framerate_st);
	if (error) {
		std::cout << "Error applying camera settings: (framerate settings)" << get_error_name(error) << std::endl;
		return;
	}
	
	framerate_st.nPortIndex = 70;
	error = OMX_SetConfig(handle, OMX_IndexConfigVideoFramerate, &framerate_st);
	if (error) {
		std::cout << "Error applying camera settings: (framerate settings preview)" << get_error_name(error) << std::endl;
		return;
	}
}

void OMXPiCamera::apply_settings(const CameraSettings& settings) {
	
	OMX_CONFIG_SHARPNESSTYPE sharpness_st;
	OMX_INIT_STRUCTURE(sharpness_st);
	sharpness_st.nPortIndex = OMX_ALL;
	sharpness_st.nSharpness = settings.sharpness;
	VideoSystemException::omx_error_check("camera settings", "sharpness", OMX_SetConfig(handle, OMX_IndexConfigCommonSharpness, &sharpness_st));
	
	OMX_CONFIG_CONTRASTTYPE contrast_st;
	OMX_INIT_STRUCTURE(contrast_st);
	contrast_st.nPortIndex = OMX_ALL;
	contrast_st.nContrast = settings.contrast;
	VideoSystemException::omx_error_check("camera settings", "contrast", OMX_SetConfig(handle, OMX_IndexConfigCommonContrast, &contrast_st));
	
	OMX_CONFIG_SATURATIONTYPE saturation_st;
	OMX_INIT_STRUCTURE(saturation_st);
	saturation_st.nPortIndex = OMX_ALL;
	saturation_st.nSaturation = settings.saturation;
	VideoSystemException::omx_error_check("camera settings", "saturation", OMX_SetConfig(handle, OMX_IndexConfigCommonSaturation, &saturation_st));
	
	OMX_CONFIG_BRIGHTNESSTYPE brightness_st;
	OMX_INIT_STRUCTURE(brightness_st);
	brightness_st.nPortIndex = OMX_ALL;
	brightness_st.nBrightness = settings.brightness;
	VideoSystemException::omx_error_check("camera settings", "brightness", OMX_SetConfig(handle, OMX_IndexConfigCommonBrightness, &brightness_st));
	
	OMX_CONFIG_EXPOSUREVALUETYPE exposure_value_st;
	OMX_INIT_STRUCTURE(exposure_value_st);
	exposure_value_st.nPortIndex = OMX_ALL;
	exposure_value_st.eMetering = settings.metering;
	exposure_value_st.xEVCompensation = (OMX_S32) ((settings.exposure_compensation << 16) / 6.0);
	exposure_value_st.nShutterSpeedMsec = settings.shutter_speed;
	exposure_value_st.bAutoShutterSpeed = settings.auto_shutter_speed;
	exposure_value_st.nSensitivity = settings.iso;
	exposure_value_st.bAutoSensitivity = settings.iso_auto;
	VideoSystemException::omx_error_check("camera settings", "exposure value", OMX_SetConfig(handle, OMX_IndexConfigCommonExposureValue, &exposure_value_st));
	
	OMX_CONFIG_EXPOSURECONTROLTYPE exposure_control_st;
	OMX_INIT_STRUCTURE(exposure_control_st);
	exposure_control_st.nPortIndex = OMX_ALL;
	exposure_control_st.eExposureControl = settings.exposure;
	VideoSystemException::omx_error_check("camera settings", "exposure control", OMX_SetConfig(handle, OMX_IndexConfigCommonExposure, &exposure_control_st));
	
	OMX_CONFIG_FRAMESTABTYPE frame_stabilization_st;
	OMX_INIT_STRUCTURE(frame_stabilization_st);
	frame_stabilization_st.nPortIndex = OMX_ALL;
	frame_stabilization_st.bStab = settings.stabilization;
	VideoSystemException::omx_error_check("camera settings", "stabilization", OMX_SetConfig(handle, OMX_IndexConfigCommonFrameStabilisation, &frame_stabilization_st));
	
	OMX_CONFIG_WHITEBALCONTROLTYPE white_balance_st;
	OMX_INIT_STRUCTURE(white_balance_st);
	white_balance_st.nPortIndex = OMX_ALL;
	white_balance_st.eWhiteBalControl = settings.white_balance;
	VideoSystemException::omx_error_check("camera settings", "white balance", OMX_SetConfig(handle, OMX_IndexConfigCommonWhiteBalance, &white_balance_st));
	
	if (!settings.white_balance) {
		OMX_CONFIG_CUSTOMAWBGAINSTYPE white_balance_gains_st;
		OMX_INIT_STRUCTURE(white_balance_gains_st);
		white_balance_gains_st.xGainR = (settings.white_balance_gain_r << 16) / 1000;
		white_balance_gains_st.xGainB = (settings.white_balance_gain_b << 16) / 1000;
		VideoSystemException::omx_error_check("camera settings", "white balance gains", OMX_SetConfig(handle, OMX_IndexConfigCustomAwbGains, &white_balance_gains_st));
	}
	
	OMX_CONFIG_IMAGEFILTERTYPE image_filter_st;
	OMX_INIT_STRUCTURE(image_filter_st);
	image_filter_st.nPortIndex = OMX_ALL;
	image_filter_st.eImageFilter = settings.filter;
	VideoSystemException::omx_error_check("camera settings", "filter", OMX_SetConfig(handle, OMX_IndexConfigCommonImageFilter, &image_filter_st));
	
	OMX_CONFIG_MIRRORTYPE mirror_st;
	OMX_INIT_STRUCTURE(mirror_st);
	mirror_st.nPortIndex = 71;
	mirror_st.eMirror = settings.mirror;
	VideoSystemException::omx_error_check("camera settings", "mirror", OMX_SetConfig(handle, OMX_IndexConfigCommonMirror, &mirror_st));
	
	OMX_CONFIG_ROTATIONTYPE rotation_st;
	OMX_INIT_STRUCTURE(rotation_st);
	rotation_st.nPortIndex = 71;
	rotation_st.nRotation = settings.rotation;
	VideoSystemException::omx_error_check("camera settings", "rotation", OMX_SetConfig(handle, OMX_IndexConfigCommonRotate, &rotation_st));
	
	OMX_CONFIG_COLORENHANCEMENTTYPE color_enhancement_st;
	OMX_INIT_STRUCTURE(color_enhancement_st);
	color_enhancement_st.nPortIndex = OMX_ALL;
	color_enhancement_st.bColorEnhancement = settings.color_enhancement;
	color_enhancement_st.nCustomizedU = settings.color_u;
	color_enhancement_st.nCustomizedV = settings.color_v;
	VideoSystemException::omx_error_check("camera settings", "color enhancement", OMX_SetConfig(handle, OMX_IndexConfigCommonColorEnhancement, &color_enhancement_st));
	
	OMX_CONFIG_BOOLEANTYPE denoise_st;
	OMX_INIT_STRUCTURE(denoise_st);
	denoise_st.bEnabled = settings.noise_reduction;
	VideoSystemException::omx_error_check("camera settings", "noise reduction", OMX_SetConfig(handle, OMX_IndexConfigStillColourDenoiseEnable, &denoise_st));
	
	OMX_CONFIG_INPUTCROPTYPE roi_st;
	OMX_INIT_STRUCTURE(roi_st);
	roi_st.nPortIndex = OMX_ALL;
	roi_st.xLeft = (settings.roi.left << 16)/100;
	roi_st.xTop = (settings.roi.top << 16)/100;
	roi_st.xWidth = (settings.roi.width << 16)/100;
	roi_st.xHeight = (settings.roi.height << 16)/100;
	VideoSystemException::omx_error_check("camera settings", "region of interest", OMX_SetConfig(handle, OMX_IndexConfigInputCropPercentages, &roi_st));
	
	OMX_CONFIG_DYNAMICRANGEEXPANSIONTYPE drc_st;
	OMX_INIT_STRUCTURE(drc_st);
	drc_st.eMode = settings.dynamic_range;
	VideoSystemException::omx_error_check("camera settings", "noise reduction", OMX_SetConfig(handle, OMX_IndexConfigDynamicRangeExpansion, &drc_st));
	
}

void OMXPiCamera::start_video_capture() {
	OMX_CONFIG_PORTBOOLEANTYPE capture_st;
	OMX_INIT_STRUCTURE(capture_st);
	capture_st.nPortIndex = 71;
	capture_st.bEnabled = OMX_TRUE;
	VideoSystemException::omx_error_check("camera", "start video", OMX_SetConfig(handle, OMX_IndexConfigPortCapturing, &capture_st));
}
