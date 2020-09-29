#include "component_event.hpp"

ComponentEvent get_event_flags(OMX_IN OMX_EVENTTYPE event, OMX_IN OMX_U32 data) {
	switch (event) {
		case OMX_EventCmdComplete:
			switch (data) {
				case OMX_CommandStateSet:
					return ComponentEvent::STATE_SET;
				case OMX_CommandPortDisable:
					return ComponentEvent::PORT_DISABLE;
				case OMX_CommandPortEnable:
					return ComponentEvent::PORT_ENABLE;
				case OMX_CommandFlush:
					return ComponentEvent::FLUSH;
				case OMX_CommandMarkBuffer:
					return ComponentEvent::MARK_BUFFER;
				default:
					break;
			}
			break;
		case OMX_EventError:
			return ComponentEvent::ERROR;
		case OMX_EventMark:
			return ComponentEvent::MARK;
		case OMX_EventPortSettingsChanged:
			return ComponentEvent::PORT_SETTINGS_CHANGED;
		case OMX_EventParamOrConfigChanged:
			return ComponentEvent::PARAM_OR_CONFIG_CHANGED;
		case OMX_EventBufferFlag:
			return ComponentEvent::BUFFER_FLAG;
		case OMX_EventResourcesAcquired:
			return ComponentEvent::RESOURCES_ACQUIRED;
		case OMX_EventDynamicResourcesAvailable:
			return ComponentEvent::DYNAMIC_RESOURCES_AVAILABLE;
		default:
			break;
	}
	return (ComponentEvent) 0;
}
