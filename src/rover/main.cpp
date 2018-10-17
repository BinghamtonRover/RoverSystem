//#define DEBUG
#define TIME

#include "camera.hpp"
#include "../network/network.hpp"

#include <cstddef>
#include <iostream>
#include <vector>
#include <cstring>
#include <cstdlib>
#ifdef TIME
	#include <chrono> // Time keeping from stdlib
	using namespace std::chrono;
#endif


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
	camera::Error err1 = camera::open(&session1, "/dev/video1", 1280, 720);
	camera::Error err2 = camera::open(&session2, "/dev/video2", 1280, 720);
	camera::Error err3 = camera::open(&session3, "/dev/video3", 1280, 720);

#ifdef DEBUG
	std::cout << "Camera 0: " << camera::get_error_string(err0) << std::endl;
	std::cout << "Camera 1: " << camera::get_error_string(err0) << std::endl;
	std::cout << "Camera 2: " << camera::get_error_string(err0) << std::endl;
	std::cout << "Camera 3: " << camera::get_error_string(err0) << std::endl;
#endif
	
	err0 = camera::start(&session0);
	err1 = camera::start(&session1);
	err2 = camera::start(&session2);
	err3 = camera::start(&session3);

#ifdef DEBUG
	std::cout << "Camera 0: " << camera::get_error_string(err0) << std::endl;
	std::cout << "Camera 1: " << camera::get_error_string(err1) << std::endl;
	std::cout << "Camera 2: " << camera::get_error_string(err2) << std::endl;
	std::cout << "Camera 3: " << camera::get_error_string(err3) << std::endl;
#endif

	// Open UDP connection
	network::Connection conn;
	network::Error net_err = network::connect(&conn, "192.168.1.1", 45545, 45545);
	if((net_err != network::Error::OK)) {
		std::cerr << "[!]Failed to connect to rover!" << std::endl;
	}

	while(true) {
#ifdef TIME
	auto start_all = high_resolution_clock::now();
#endif
		//Grab a frame
		uint8_t* frame_buffer;
		std::size_t frame_size;
		err1 = camera::grab_frame(&session0, &frame_buffer, &frame_size);
		frame_counter ++;
#ifdef DEBUG
		// Print camera buffer info
		std::cout << (void*)frame_buffer << std::endl;
		std::cout << "1" << camera::get_error_string(err1) << std::endl;
#endif

#ifdef TIME
	auto end_grab_frame = high_resolution_clock::now();
	std::cout << 
		"[LOCAL_TIME] Time to grab frame: " <<
		duration_cast<milliseconds>(end_grab_frame - start_all).count() <<
		"ms" <<
	std::endl;
#endif



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

		// Camera Frames
		//Temporary 'for' loop to use the same frame from session0 for all streams
		for (unsigned int i = 0; i < 4; i++) {
			uint8_t num_buffers = (frame_size/CAMERA_MESSAGE_SIZE) + 1;
			for (unsigned int j = 0; j < num_buffers; j++) {
				network::Buffer* camera_buffer = network::get_outgoing_buffer();
				// If we are not at the last buffer
				uint16_t buffer_size = (j != num_buffers-1)?CAMERA_MESSAGE_SIZE:frame_size%CAMERA_MESSAGE_SIZE;
				network::CameraMessage message = {
					static_cast<uint8_t>(i),              // stream_index
					static_cast<uint16_t>(frame_counter), // frame_index
					static_cast<uint8_t>(j),              // section_index
					num_buffers,                          // section_count
					buffer_size,                          // size
					nullptr                               // data
				};
				message.data = (uint8_t*) malloc(CAMERA_MESSAGE_SIZE);
				// OPTIMIZE: reuse the same message object to avoid reallocating memory
				memcpy(message.data, frame_buffer+CAMERA_MESSAGE_SIZE*j, CAMERA_MESSAGE_SIZE);
				network::serialize(camera_buffer, &message);

				network::queue_outgoing(&conn, network::MessageType::CAMERA, camera_buffer);
			}
		}

		// Send iiiitttttt //
		network::drain_outgoing(&conn);
		
		/// Return all buffers
		err1 = camera::return_buffer(&session1, frame_buffer);


#ifdef DEBUG
		std::cout << "2" << camera::get_error_string(err1) << std::endl;
#endif
#ifdef TIME
		auto end_all = high_resolution_clock::now();
		std::cout << 
			"[LOCAL_TIME] Time to send all logging and camera buffers: " <<
			duration_cast<milliseconds>(end_all - end_grab_frame).count() <<
			"ms" <<
		std::endl;
#endif
	}


	/// Destruct Rover objects
	// Close the camera session
	camera::close(&session0);
	camera::close(&session1);
	camera::close(&session2);
	camera::close(&session3);
	return 0;
}
