#include <IL/OMX_Broadcom.h>
#include <interface/vcos/vcos.h>

class OMXVideoComponent {
private:
	void wait_event(VCOS_UNSIGNED events, VCOS_UNSIGNED *retrieved_events);
	void received_event(OMX_IN OMX_EVENTTYPE event, OMX_IN OMX_U32 event_data1, OMX_IN OMX_U32 event_data2);
	
	bool initialized = false;
	
	static OMX_CALLBACKTYPE callbacks;
public:
	~OMXVideoComponent();
	bool init();
	void deinit();
	static OMX_ERRORTYPE event_handler(
		OMX_IN OMX_HANDLETYPE comp, 
		OMX_IN OMX_PTR app_data, 
		OMX_IN OMX_EVENTTYPE event, 
		OMX_IN OMX_U32 data1, 
		OMX_IN OMX_U32 data2, 
		OMX_IN OMX_PTR event_data);
	
	// TODO: Better encapsulation for these types
	OMX_STRING name;
	OMX_HANDLETYPE handle;
	VCOS_EVENT_FLAGS_T flags;
};
