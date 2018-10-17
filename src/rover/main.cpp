#include "camera.hpp"
#include "../network/network.hpp"

#include <stddef.h>
#include <iostream>
#include <vector>
#include <cstring>

const unsigned int CAMERA_MESSAGE_SIZE = 65000;

int main() {
	/// Initialization of Rover objects
	unsigned int frame_counter = 0;
	// OPTIMIZE: Consider making a rover class with an iteratable list of cameras
	// Open all cameras
	camera::CaptureSession session0;
	camera::CaptureSession session1;
	camera::CaptureSession session2;
	camera::CaptureSession session3;

	camera::Error err0 = camera::open(&session0, "/dev/video0", 1280, 720);
	std::cout << "Camera 0: " << camera::get_error_string(err0) << std::endl;
	camera::Error err1 = camera::open(&session1, "/dev/video1", 1280, 720);
	std::cout << "Camera 1: " << camera::get_error_string(err0) << std::endl;
	camera::Error err2 = camera::open(&session2, "/dev/video2", 1280, 720);
	std::cout << "Camera 2: " << camera::get_error_string(err0) << std::endl;
	camera::Error err3 = camera::open(&session3, "/dev/video3", 1280, 720);
	std::cout << "Camera 3: " << camera::get_error_string(err0) << std::endl;

	err0 = camera::start(&session0);
	std::cout << "Camera 0: " << camera::get_error_string(err0) << std::endl;
	err1 = camera::start(&session1);
	std::cout << "Camera 1: " << camera::get_error_string(err1) << std::endl;
	err2 = camera::start(&session2);
	std::cout << "Camera 2: " << camera::get_error_string(err2) << std::endl;
	err3 = camera::start(&session3);
	std::cout << "Camera 3: " << camera::get_error_string(err3) << std::endl;

	// Open UDP connection
	network::Connection conn;
	network::Error net_err = network::connect(&conn, "192.168.1.1", 45545, 45545);
	if((net_err != network::Error::OK)) {
		std::cerr << "[!]Failed to connect to rover!" << std::endl;
	}

	while(true) {
		//Grab a frame
		uint8_t* frame_buffer;
		std::size_t frame_size;
		err1 = camera::grab_frame(&session1, &frame_buffer, &frame_size);
		std::cout << "1" << camera::get_error_string(err1) << std::endl;
		err1 = camera::return_buffer(&session1, frame_buffer);
		std::cout << "2" << camera::get_error_string(err1) << std::endl;
		frame_counter ++;



		/// Network
		network::poll_incoming(&conn);
		network::Message message;
		// Receive incoming messages //
		while(network::dequeue_incoming(&conn, &message)) {
				switch(message.type) {
					case network::MessageType::HEARTBEAT:
					{
						network::Buffer* outgoing = network::get_outgoing_buffer();
						network::queue_outgoing(&conn, network::MessageType::HEARTBEAT, outgoing);
						break;
					}
					default:
						break;
				}
		}
		// Sample code to send outgoing packets //
		// Log messages
		network::Buffer* log_buffer = network::get_outgoing_buffer();
		char s[] = "[LOG] Hewwo Wowld!";
		network::LogMessage log_message = {static_cast<uint8_t>(strlen(s)), s};
		network::serialize(log_buffer, &log_message);

		network::queue_outgoing(&conn, network::MessageType::LOG, log_buffer);

		// TODO: Camera frames
		// For each camera frame:
		//     Get the required number of buffers by FRAME_SIZE/BUFFER_SIZE + 1
		//     For each buffer:
		//         Serialize the buffer with data from
		//             CAMERA_FRAME_BASE_POINTER + BUFFER_SIZE*i
		//         Queue outgoing buffer
		
		// Temporary for loop to use the same one frame for all streams
		for (unsigned int i = 0; i < 4; i++) {
			uint8_t num_buffers = (frame_size/CAMERA_MESSAGE_SIZE) + 1;
			std::vector<network::Buffer*> buffers(num_buffers);
			buffers.push_back(network::get_outgoing_buffer());
			for (unsigned int j = 0; j < buffers.size(); j++) {
				// If we are not at the last buffer
				uint16_t buffer_size = (j != buffers.size()-1)?CAMERA_MESSAGE_SIZE:frame_size%CAMERA_MESSAGE_SIZE;
				network::CameraMessage message = {
					static_cast<uint8_t>(i),              // stream_index
					static_cast<uint16_t>(frame_counter), // frame_index
					static_cast<uint8_t>(j),              // section_index
					num_buffers,                          // section_count
					buffer_size,                          // size
					nullptr                               // data
				};
				memcpy(message.data, frame_buffer+CAMERA_MESSAGE_SIZE*j, CAMERA_MESSAGE_SIZE);
				network::serialize(buffers[j], &message);

				network::queue_outgoing(&conn, network::MessageType::CAMERA, buffers[j]);
			}
		}



		// Send iiiitttttt
		network::drain_outgoing(&conn);
		
	}
	/// Destruct Rover objects
	// Close the camera session
	camera::close(&session0);
	camera::close(&session1);
	camera::close(&session2);
	camera::close(&session3);
	return 0;
}
