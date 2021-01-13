#include <iostream>
#include <fstream>
#include <chrono>
#include <climits>
#include <vector>
#include <algorithm>
#include <string>

#include "video_system.hpp"
#include "video_system_exception.hpp"

/*
	This example records 3 seconds of encoded video and outputs to a file
	The calls to VideoSystem are asynchronous so that other work can be done in the same thread
	
	The file needs a container before you can play it. Install MKVToolNix and use this command:
		mkvmerge --default-duration 0:30p -o video.mkv video.h264
 */

int main(int argc, char** argv) {
	std::ofstream video_out_file;
	video_out_file.open("video.h264");
	
	try {
		
		VideoSystem video_system;
		video_system.init();	// Load drivers, connect camera and encoder together
		video_system.start_video();	// Activate the video components (does not output anything)

		int n_seconds;
		if (argc == 2) {
			n_seconds = std::stoi(argv[1]);
		} else {
			n_seconds = 3;
		}

		std::cout << "Recording for " << n_seconds << " seconds." << std::endl;

		// Keep recording frames for n_seconds seconds
		std::chrono::time_point<std::chrono::system_clock> end_time = std::chrono::system_clock::now() + std::chrono::seconds(n_seconds);

		// Start filling before the main loop
		video_system.fill_partial_frame_async();
		for (;;) {
			// The loop runs while the buffer fills
			// Check if the buffer is filled each time, only process if it is
			if (video_system.partial_frame_available_async()) {
				OMX_BUFFERHEADERTYPE *frame;
				video_system.get_partial_frame_async(&frame);

				video_out_file.write( (char*)frame->pBuffer, frame->nFilledLen );

				// Start filling the buffer again
				video_system.fill_partial_frame_async();
			}

			// .... do other tasks ....
			
			if (std::chrono::system_clock::now() >= end_time) break;
		}
		
		video_system.stop_video();	// Stop the video components (makes idle)
		video_system.deinit();	// Deinitialize components, OMX, drivers
		
		video_out_file.close();
		std::cout << "Recorded to video.h264. To make playable, use:\n\tmkvmerge --default-duration 0:30p -o video.mkv video.h264" << std::endl;
		
		return 0;
		
	} catch (const VideoSystemException& e) {
		std::cout << "Caught an exception: " << e.get_details() << std::endl;
		video_out_file.close();
		return 1;
	}
	
}
