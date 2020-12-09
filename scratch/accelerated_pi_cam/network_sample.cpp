#include <iostream>
#include <fstream>
#include <chrono>
#include <climits>
#include <vector>
#include <algorithm>

#include <network.hpp>

#include "video_system.hpp"
#include "video_system_exception.hpp"

/*
 *	This example records 3 seconds of encoded video and publishes continuously
 * 	to the network.
 */

std::vector<network::CSICameraMessage> messages;
size_t expected_size = 0;

void simulate_publish(network::CSICameraMessage& msg) {
	// For simulation only, not recommended to implement anything like this
	expected_size += msg.size;
	network::CSICameraMessage cp;
	cp.size = msg.size;
	cp.section_index = msg.section_index;
	cp.stream_index = msg.stream_index;
	cp.frame_index = msg.frame_index;
	if (msg.size > 0) {
		uint8_t* in_buf = new uint8_t[msg.size];
		for (uint16_t i = 0; i < msg.size; ++i) {
			in_buf[i] = msg.data[i];
		}
		cp.data = in_buf;
	}
	
	messages.push_back(cp);
	
}

void simulate_receive_stream(std::ostream& out) {
	std::vector<network::CSICameraMessage> section;
	std::cout << "Simulating receive of " << messages.size() << " messages.\n";
	for (auto it = messages.begin(); it != messages.end(); ++it) {
		if ((*it).size > 0) {
			section.push_back(*it);
		} else {
			auto jt = section.begin();
			if ((*jt).section_index != 0) std::cerr << "Error: section does not start with 0 index (starts with " << (int)(*jt).section_index << ")." << std::endl;
			for (; jt != section.end(); ++jt) {
				out.write((char*)(*jt).data, (*jt).size);
				delete[] (*jt).data;
			}
			section.clear();
		}
	}
}

int main() {
	
	std::ofstream video_out_file;
	video_out_file.open("netvid-ctrl.h264");

	try {
		
		VideoSystem video_system;
		video_system.init();	// Load drivers, connect camera and encoder together
		video_system.start_video();	// Activate the video components (does not output anything)
		
		VideoSystem::MessageBuilder message_builder;	// Helps with splitting frames into sections
		
		// Keep recording frames for 3 seconds
		std::chrono::time_point<std::chrono::system_clock> end_time = std::chrono::system_clock::now() + std::chrono::seconds(3);
		for (;;) {
			OMX_BUFFERHEADERTYPE *frame;
			network::CSICameraMessage msg;
			
			bool starting_new_frame = video_system.get_partial_frame(&frame);
			if (starting_new_frame && std::chrono::system_clock::now() >= end_time) break;
			if (starting_new_frame) {
				if (message_builder.get_frame_delimiter(msg)) {
					simulate_publish(msg);	// Old frame delimited by a size 0 message (setup by start_new_frame)
				}
			}
			
			message_builder.start_partial_frame(frame);
			
			while (message_builder.available()) {
				message_builder.fill_message(msg);
				simulate_publish(msg);
			}
			video_out_file.write( (char*)frame->pBuffer, frame->nFilledLen );
		}
		video_system.stop_video();	// Stop the video components (makes idle)
		video_system.deinit();	// Deinitialize components, OMX, drivers

		video_out_file.close();
		
	} catch (const VideoSystemException& e) {
		std::cout << "Caught an exception: " << e.get_details() << std::endl;
		return 1;
	}
	
	std::cout << "Finished recording. Simulating receive...\n";
	std::ofstream out_file;
	out_file.open("netvid.h264");
	simulate_receive_stream(out_file);
	out_file.close();
	std::cout << "Wrote two files: netvid-ctrl.h264, netvid.h264\n";
	std::cout << "Expected file size = " << expected_size << std::endl;
	std::cout << "(Note: netvid likely 1 frame shorter due to simulation)\n";
	return 0;
	
}
