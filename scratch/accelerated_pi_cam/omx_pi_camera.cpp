#include "omx_pi_camera.hpp"

#include <IL/OMX_Broadcom.h>
#include <iostream>

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
