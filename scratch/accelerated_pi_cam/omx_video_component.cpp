#include "omx_video_component.hpp"
#include <iostream>

OMXVideoComponent::~OMXVideoComponent() {
	if (initialized) deinit();
}

OMX_CALLBACKTYPE OMXVideoComponent::callbacks = { EventHandler: event_handler, FillBufferDone: nullptr };

void OMXVideoComponent::wait_event(VCOS_UNSIGNED events, VCOS_UNSIGNED *retrieved_events) {
	// 0x1 is an enum for EVENT_ERROR in example, still figuring this out
	VCOS_UNSIGNED set;
	vcos_event_flags_get(&flags, events | 0x1, VCOS_OR_CONSUME, VCOS_SUSPEND, &set);
	
	if (retrieved_events != nullptr) {
		*retrieved_events = set;
	}
}

OMX_ERRORTYPE OMXVideoComponent::event_handler(OMX_IN OMX_HANDLETYPE comp, OMX_IN OMX_PTR app_data, 
		OMX_IN OMX_EVENTTYPE event, OMX_IN OMX_U32 data1, OMX_IN OMX_U32 data2, OMX_IN OMX_PTR event_data) {
	OMXVideoComponent *component = (OMXVideoComponent*) app_data;
	component->received_event(event, data1, data2);
	return OMX_ErrorNone;
}

void OMXVideoComponent::received_event(OMX_IN OMX_EVENTTYPE event, OMX_IN OMX_U32 event_data1, OMX_IN OMX_U32 event_data2) {
	// TODO: Total implementation needed
}

bool OMXVideoComponent::init() {
	if (vcos_event_flags_create(&flags, "component")) {
		return false;
	}
	
	if (OMX_GetHandle(&handle, name, this, &callbacks)) {
		return false;
	}
	
	OMX_INDEXTYPE types[] = { OMX_IndexParamAudioInit, OMX_IndexParamVideoInit, OMX_IndexParamImageInit, OMX_IndexParamOtherInit };
	OMX_PORT_PARAM_TYPE ports;
	
	for (int i = 0; i < sizeof(types); ++i) {
		if (OMX_GetParameter(handle, types[i], &ports)) {
			return false;
		}
		
		// From the example: I'm not sure why all ports get disabled
		for (OMX_U32 p = ports.nStartPortNumber; p < ports.nStartPortNumber + ports.nPorts; ++p) {
			OMX_SendCommand(handle, OMX_CommandPortDisable, p, 0);
			// 0x4 is EVENT_PORT_DISABLE in example. TODO: enum, existing definition, something other than hex that tells you nothing
			wait_event(0x4, nullptr);
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
