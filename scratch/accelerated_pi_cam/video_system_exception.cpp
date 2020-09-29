#include "video_system_exception.hpp"

#define DUMP_CASE(x) case x: return #x;

VideoSystemException::VideoSystemException(const char *component, const char *activity, OMX_ERRORTYPE error) {
	this->component = component;
	this->activity = activity;
	this->error = error;
}

std::string VideoSystemException::get_details() const {
	std::string buf("Error from ");
	buf += component;
	buf += '(';
	buf += activity;
	buf += "): ";
	buf += get_error_name(error);
	return buf;
}

void VideoSystemException::omx_error_check(const char *component, const char *activity, OMX_ERRORTYPE error) {
	if (error) {
		throw VideoSystemException(component, activity, error);
	}
}

const char * VideoSystemException::get_error_name(OMX_ERRORTYPE error) {
	switch (error){
		DUMP_CASE (OMX_ErrorNone)
		DUMP_CASE (OMX_ErrorInsufficientResources)
		DUMP_CASE (OMX_ErrorUndefined)
		DUMP_CASE (OMX_ErrorInvalidComponentName)
		DUMP_CASE (OMX_ErrorComponentNotFound)
		DUMP_CASE (OMX_ErrorInvalidComponent)
		DUMP_CASE (OMX_ErrorBadParameter)
		DUMP_CASE (OMX_ErrorNotImplemented)
		DUMP_CASE (OMX_ErrorUnderflow)
		DUMP_CASE (OMX_ErrorOverflow)
		DUMP_CASE (OMX_ErrorHardware)
		DUMP_CASE (OMX_ErrorInvalidState)
		DUMP_CASE (OMX_ErrorStreamCorrupt)
		DUMP_CASE (OMX_ErrorPortsNotCompatible)
		DUMP_CASE (OMX_ErrorResourcesLost)
		DUMP_CASE (OMX_ErrorNoMore)
		DUMP_CASE (OMX_ErrorVersionMismatch)
		DUMP_CASE (OMX_ErrorNotReady)
		DUMP_CASE (OMX_ErrorTimeout)
		DUMP_CASE (OMX_ErrorSameState)
		DUMP_CASE (OMX_ErrorResourcesPreempted)
		DUMP_CASE (OMX_ErrorPortUnresponsiveDuringAllocation)
		DUMP_CASE (OMX_ErrorPortUnresponsiveDuringDeallocation)
		DUMP_CASE (OMX_ErrorPortUnresponsiveDuringStop)
		DUMP_CASE (OMX_ErrorIncorrectStateTransition)
		DUMP_CASE (OMX_ErrorIncorrectStateOperation)
		DUMP_CASE (OMX_ErrorUnsupportedSetting)
		DUMP_CASE (OMX_ErrorUnsupportedIndex)
		DUMP_CASE (OMX_ErrorBadPortIndex)
		DUMP_CASE (OMX_ErrorPortUnpopulated)
		DUMP_CASE (OMX_ErrorComponentSuspended)
		DUMP_CASE (OMX_ErrorDynamicResourcesUnavailable)
		DUMP_CASE (OMX_ErrorMbErrorsInFrame)
		DUMP_CASE (OMX_ErrorFormatNotDetected)
		DUMP_CASE (OMX_ErrorContentPipeOpenFailed)
		DUMP_CASE (OMX_ErrorContentPipeCreationFailed)
		DUMP_CASE (OMX_ErrorSeperateTablesUsed)
		DUMP_CASE (OMX_ErrorTunnelingUnsupported)
		DUMP_CASE (OMX_ErrorDiskFull)
		DUMP_CASE (OMX_ErrorMaxFileSize)
		DUMP_CASE (OMX_ErrorDrmUnauthorised)
		DUMP_CASE (OMX_ErrorDrmExpired)
		DUMP_CASE (OMX_ErrorDrmGeneral)
		default: return "unknown OMX_ERRORTYPE";
	}
}
