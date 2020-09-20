#ifndef VIDEO_SYSTEM_EXCEPTION_H
#define VIDEO_SYSTEM_EXCEPTION_H

#include <stdexcept>
#include <string>
#include <IL/OMX_Broadcom.h>

class VideoSystemException : public std::exception {
public:
	VideoSystemException(const char *component, const char *activity, OMX_ERRORTYPE error);
	
	std::string get_details() const;
	
	const char * what() const throw() {

		return "Error in video system";
	}
	
	static const char * get_error_name(OMX_ERRORTYPE error);
	static void omx_error_check(const char *component, const char *activity, OMX_ERRORTYPE error);
private:
	const char *component;
	const char *activity;
	OMX_ERRORTYPE error;
};

#endif
