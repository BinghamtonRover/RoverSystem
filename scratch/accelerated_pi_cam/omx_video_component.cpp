#include "omx_video_component.hpp"
#include <iostream>

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
