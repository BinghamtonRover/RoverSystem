#include "omx_video_component.hpp"

#include <bcm_host.h>
#include <iostream>

#include "video_system_exception.hpp"

#define DUMP_CASE(x) case x: return #x;

#define OMX_INIT_STRUCTURE(x) \
  memset (&(x), 0, sizeof (x)); \
  (x).nSize = sizeof (x); \
  (x).nVersion.nVersion = OMX_VERSION; \
  (x).nVersion.s.nVersionMajor = OMX_VERSION_MAJOR; \
  (x).nVersion.s.nVersionMinor = OMX_VERSION_MINOR; \
  (x).nVersion.s.nRevision = OMX_VERSION_REVISION; \
  (x).nVersion.s.nStep = OMX_VERSION_STEP

OMXVideoComponent::~OMXVideoComponent() {
	if (initialized) deinit();
}

OMX_CALLBACKTYPE OMXVideoComponent::callbacks = { EventHandler: event_handler, FillBufferDone: fill_buffer_done };

void OMXVideoComponent::wait_event(VCOS_UNSIGNED events, VCOS_UNSIGNED *retrieved_events) {
	VCOS_UNSIGNED set;
	vcos_event_flags_get(&flags, events | ComponentEvent::ERROR, VCOS_OR_CONSUME, VCOS_SUSPEND, &set);
	
	if (retrieved_events != nullptr) {
		*retrieved_events = set;
	}
}

OMX_ERRORTYPE OMXVideoComponent::event_handler(OMX_IN OMX_HANDLETYPE comp, OMX_IN OMX_PTR app_data, 
		OMX_IN OMX_EVENTTYPE event, OMX_IN OMX_U32 data1, OMX_IN OMX_U32 data2, OMX_IN OMX_PTR event_data) {
	OMXVideoComponent *component = (OMXVideoComponent*) app_data;
	
	component->received_event(get_event_flags(event, data1));
	return OMX_ErrorNone;
}

OMX_ERRORTYPE OMXVideoComponent::fill_buffer_done(OMX_IN OMX_HANDLETYPE comp, OMX_IN OMX_PTR app_data, 
	OMX_IN OMX_BUFFERHEADERTYPE* buffer){
	OMXVideoComponent *component = (OMXVideoComponent*) app_data;
	component->received_event(ComponentEvent::FILL_BUFFER_DONE);

	return OMX_ErrorNone;
}

void OMXVideoComponent::received_event(ComponentEvent event) {
	vcos_event_flags_set(&flags, event, VCOS_OR);
}

bool OMXVideoComponent::init() {
	if (vcos_event_flags_create(&flags, "component")) {
		std::cout << "Component error: vcos_event_flags_create" << std::endl;
		return false;
	}
	
	OMX_ERRORTYPE error = OMX_GetHandle(&handle, name, this, &callbacks);
	if (error) {
		std::cout << "Component error: GetHandle: " << get_error_name(error) << std::endl;
		return false;
	}
	
	OMX_INDEXTYPE types[] = { OMX_IndexParamAudioInit, OMX_IndexParamVideoInit, OMX_IndexParamImageInit, OMX_IndexParamOtherInit };
	OMX_PORT_PARAM_TYPE ports;
	
	OMX_INIT_STRUCTURE(ports);
	
	for (size_t i = 0; i < 4; ++i) {
		if ((error = OMX_GetParameter(handle, types[i], &ports))) {
			std::cout << "Component error: GetParameter: " << get_error_name(error) << std::endl;
			return false;
		}
		
		// From the example: I'm not sure why all ports get disabled
		for (OMX_U32 p = ports.nStartPortNumber; p < ports.nStartPortNumber + ports.nPorts; ++p) {
			OMX_SendCommand(handle, OMX_CommandPortDisable, p, 0);
			wait_event(ComponentEvent::PORT_DISABLE, nullptr);
		}
	}
	initialized = true;
	return true;
}

void OMXVideoComponent::deinit() {
	initialized = false;
	vcos_event_flags_delete(&flags);
	OMX_FreeHandle(handle);
}

void OMXVideoComponent::change_state(OMX_STATETYPE state) {
	VideoSystemException::omx_error_check("component", "change state", OMX_SendCommand(handle, OMX_CommandStateSet, state, 0));
}

void OMXVideoComponent::enable_port(OMX_U32 port) {
	VideoSystemException::omx_error_check("component", "enable port", OMX_SendCommand(handle, OMX_CommandPortEnable, port, 0));
}

void OMXVideoComponent::disable_port(OMX_U32 port) {
	VideoSystemException::omx_error_check("component", "disable port", OMX_SendCommand(handle, OMX_CommandPortDisable, port, 0));
}

const char* OMXVideoComponent::get_error_name(OMX_ERRORTYPE error) {
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
