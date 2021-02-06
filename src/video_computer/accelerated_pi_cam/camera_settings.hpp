#ifndef CAMERA_SETTINGS_H
#define CAMERA_SETTINGS_H

#include <IL/OMX_Broadcom.h>

class CameraSettings {
public:
	OMX_U32 width = 1920;
	OMX_U32 height = 1080;
	OMX_U32 framerate = 30;
	OMX_U32 bitrate = 17000000;
	OMX_S32 sharpness = 0;
	OMX_S32 contrast = 0;
	OMX_S32 saturation = 0;
	OMX_U32 brightness = 50;
	OMX_METERINGTYPE metering = OMX_MeteringModeAverage;
	OMX_S32 exposure_compensation = 0;
	OMX_U32 shutter_speed = 125;
	OMX_BOOL auto_shutter_speed = OMX_TRUE;
	OMX_U32 iso = 100;
	OMX_BOOL iso_auto = OMX_TRUE;
	OMX_EXPOSURECONTROLTYPE exposure = OMX_ExposureControlAuto;
	OMX_BOOL stabilization = OMX_FALSE;
	OMX_WHITEBALCONTROLTYPE white_balance = OMX_WhiteBalControlAuto;
	OMX_U32 white_balance_gain_r = 1000;
	OMX_U32 white_balance_gain_b = 1000;
	OMX_IMAGEFILTERTYPE filter = OMX_ImageFilterNone;
	OMX_MIRRORTYPE mirror = OMX_MirrorNone;
	OMX_S32 rotation = 0;
	OMX_BOOL color_enhancement = OMX_FALSE;
	OMX_U8 color_u = 128;
	OMX_U8 color_v = 128;
	OMX_BOOL noise_reduction = OMX_TRUE;
	struct {
		OMX_U32 top = 0;
		OMX_U32 left = 0;
		OMX_U32 width = 100;
		OMX_U32 height = 100;
	} roi;
	OMX_DYNAMICRANGEEXPANSIONMODETYPE dynamic_range = OMX_DynRangeExpOff;
	
};

#endif
