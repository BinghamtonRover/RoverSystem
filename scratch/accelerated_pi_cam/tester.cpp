#include <iostream>
#include <fstream>
#include <chrono>

#include "video_encoder.hpp"
#include "video_system_exception.hpp"

/*
	This example records 3 seconds of encoded video and outputs to a file
	The file needs a container before you can play it. Install MKVToolNix and use this command:
		mkvmerge --default-duration 0:30p -o video.mkv video.h264
 */

int main() {
	std::ofstream video_out_file;
	video_out_file.open("video.h264");
	
	try {
		
		VideoEncoder video_system;
		video_system.init();	// Load drivers, connect camera and encoder together
		video_system.start_video();	// Activate the video components (does not output anything)
	
		// Keep recording frames for 3 seconds
		std::chrono::time_point<std::chrono::system_clock> end_time = std::chrono::system_clock::now() + std::chrono::seconds(3);
		for (;;) {
			OMX_BUFFERHEADERTYPE *frame = video_system.get_frame();	// Encode the latest frame AND get the buffer
			
			video_out_file.write( (char*)frame->pBuffer, frame->nFilledLen );
			
			if (std::chrono::system_clock::now() >= end_time) break;
		}
		video_out_file.close();
		return 0;
		
	} catch (const VideoSystemException& e) {
		std::cout << "Caught an exception: " << e.get_details() << std::endl;
		video_out_file.close();
		return 1;
	}
	
}
