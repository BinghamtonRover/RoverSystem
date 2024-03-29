#ifndef OMX_VIDEO_COMPONENT_H
#define OMX_VIDEO_COMPONENT_H

#include <IL/OMX_Broadcom.h>
#include <interface/vcos/vcos.h>

#include "component_event.hpp"

class OMXVideoComponent {
private:
	void received_event(ComponentEvent event);
	
	bool initialized = false;
	
	static OMX_CALLBACKTYPE callbacks;
public:
	
	~OMXVideoComponent();
	void init();
	void deinit();
	void wait_event(VCOS_UNSIGNED events, VCOS_UNSIGNED *retrieved_events);
	inline void wait_event(VCOS_UNSIGNED events) { wait_event(events, 0); }
	void change_state(OMX_STATETYPE state);
	void enable_port(OMX_U32 port);
	void disable_port(OMX_U32 port);
	static OMX_ERRORTYPE event_handler(
		OMX_IN OMX_HANDLETYPE comp, 
		OMX_IN OMX_PTR app_data, 
		OMX_IN OMX_EVENTTYPE event, 
		OMX_IN OMX_U32 data1, 
		OMX_IN OMX_U32 data2, 
		OMX_IN OMX_PTR event_data
	);
	static OMX_ERRORTYPE fill_buffer_done(
		OMX_IN OMX_HANDLETYPE comp, 
		OMX_IN OMX_PTR app_data, 
		OMX_IN OMX_BUFFERHEADERTYPE* buffer
	);
	static const char* get_error_name(OMX_ERRORTYPE error);
	// TODO: Better encapsulation for these types
	OMX_STRING name;
	OMX_HANDLETYPE handle;
	VCOS_EVENT_FLAGS_T flags;
};

#endif
