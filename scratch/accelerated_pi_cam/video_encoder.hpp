#ifndef VIDEO_ENCODER_H
#define VIDEO_ENCODER_H

#include "omx_video_component.hpp"

// Macro is from the example
#define OMX_INIT_STRUCTURE(x) \
  memset (&(x), 0, sizeof (x)); \
  (x).nSize = sizeof (x); \
  (x).nVersion.nVersion = OMX_VERSION; \
  (x).nVersion.s.nVersionMajor = OMX_VERSION_MAJOR; \
  (x).nVersion.s.nVersionMinor = OMX_VERSION_MINOR; \
  (x).nVersion.s.nRevision = OMX_VERSION_REVISION; \
  (x).nVersion.s.nStep = OMX_VERSION_STEP

class VideoEncoder {
private:
	OMXVideoComponent camera;
	OMXVideoComponent encoder;
	OMXVideoComponent null_sink;
	
	void load_camera_drivers();
public:
	VideoEncoder();
	//~VideoEncoder();
	
	// TODO: Options for multiplexed cameras, picture settings
	void init_camera_system();
};

#endif
