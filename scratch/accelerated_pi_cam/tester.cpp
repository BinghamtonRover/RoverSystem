#include <iostream>
#include <fstream>
#include <chrono>
#include <climits>
#include <vector>
#include <algorithm>

#include "video_system.hpp"
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
		
		VideoSystem video_system;
		video_system.init();	// Load drivers, connect camera and encoder together
		video_system.start_video();	// Activate the video components (does not output anything)
		unsigned long counter = 0;
		unsigned long min = ULONG_MAX;
		unsigned long max = 0;
		unsigned long sum = 0;
		unsigned long start_codes = 0;
		std::vector<char> first_elems;
		std::vector<char> last_elems;
		std::vector<unsigned long> first_bufs;
		// Keep recording frames for 3 seconds
		std::chrono::time_point<std::chrono::system_clock> end_time = std::chrono::system_clock::now() + std::chrono::seconds(3);
		for (;;) {
			OMX_BUFFERHEADERTYPE *frame = video_system.get_partial_frame();	// Encode the latest frame AND get the buffer
			if (first_bufs.size() < 30) first_bufs.push_back(frame->nFilledLen);
			if (frame->nFilledLen < min) min = frame->nFilledLen;
			if (frame->nFilledLen > max) max = frame->nFilledLen;
			if (std::find(first_elems.begin(), first_elems.end(), ((char*)(frame->pBuffer))[0]) == first_elems.end() && frame->nFilledLen == 65536) {
				first_elems.push_back(((char*)(frame->pBuffer))[0]);
			}
			if (std::find(last_elems.begin(), last_elems.end(), ((char*)(frame->pBuffer))[frame->nFilledLen-1]) == last_elems.end() && frame->nFilledLen != 65536 ) {
				last_elems.push_back(((char*)(frame->pBuffer))[frame->nFilledLen-1]);
			}
			if (frame->nFilledLen >= 3 && frame->pBuffer[0] == 0 && frame->pBuffer[1] == 0 && frame->pBuffer[2] == 0 && frame->pBuffer[3] == 1) {
				start_codes += 1;
			}
			sum += frame->nFilledLen;
			++counter;
			video_out_file.write( (char*)frame->pBuffer, frame->nFilledLen );
			
			if (std::chrono::system_clock::now() >= end_time) break;
		}
		
		video_system.stop_video();	// Stop the video components (makes idle)
		video_system.deinit();	// Deinitialize components, OMX, drivers
		
		video_out_file.close();
		
		std::cout << "SUMMARY:"
			<< "\n\tTotal  operations = " << counter
			<< "\n\tMinimum buffer sz = " << min
			<< "\n\tMaximum buffer sz = " << max
			<< "\n\tAverage buffer sz = " << sum / counter
			<< "\n\tTotal start codes = " << start_codes
			<< "\n\tAll  first  elems = (" << first_elems.size() << "){ ";
			for (const char& c : first_elems) std::cout << (int)c << ' '
			<< "\n\tFirst  buf  sizes = (" << first_bufs.size() << "){ ";
			for (unsigned long& l : first_bufs) std::cout << l << ' ';
			std::cout << "}\nEND OF SUMMARY." << std::endl;
			
		
		return 0;
		
	} catch (const VideoSystemException& e) {
		std::cout << "Caught an exception: " << e.get_details() << std::endl;
		video_out_file.close();
		return 1;
	}
	
}
