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


/*
 * GLOBAL CONSTANTS
 */
const unsigned int CAMERA_MESSAGE_SIZE = 65000;
const unsigned int WEBCAM_1_WIDTH = 1280;
const unsigned int WEBCAM_1_HEIGHT = 720;


int main() {
	/*
	 * INITILIZATION: Rover Objects
	 * OPTIMIZE: Consider making a rover class with an iteratable list of cameras
	 */
	unsigned int frame_counter = 0;
	// Camera streams
	camera::CaptureSession session0;
	camera::CaptureSession session1;
	camera::CaptureSession session2;
	camera::CaptureSession session3;
	// UPD connection
	network::Connection conn;

	// Open all cameras
	{
		camera::Error err0 = camera::open(&session0, "/dev/video0", WEBCAM_1_WIDTH, WEBCAM_1_HEIGHT);
		if(err0 != camera::Error::OK) std::cout << "Camera 0: " << camera::get_error_string(err0) << std::endl;

		camera::Error err1 = camera::open(&session1, "/dev/video1", WEBCAM_1_WIDTH, WEBCAM_1_HEIGHT);
		if(err1 != camera::Error::OK) std::cout << "Camera 1: " << camera::get_error_string(err1) << std::endl;

		camera::Error err2 = camera::open(&session2, "/dev/video2", WEBCAM_1_WIDTH, WEBCAM_1_HEIGHT);
		if(err2 != camera::Error::OK) std::cout << "Camera 2: " << camera::get_error_string(err2) << std::endl;

		camera::Error err3 = camera::open(&session3, "/dev/video3", WEBCAM_1_WIDTH, WEBCAM_1_HEIGHT);
		if(err3 != camera::Error::OK) std::cout << "Camera 3: " << camera::get_error_string(err3) << std::endl;
	}
	// Start cameras
	{
		camera::Error err0 = camera::start(&session0);
		if(err0 != camera::Error::OK) std::cout << "Camera 0: " << camera::get_error_string(err0) << std::endl;

		camera::Error err1 = camera::start(&session1);
		if(err1 != camera::Error::OK) std::cout << "Camera 1: " << camera::get_error_string(err1) << std::endl;

		camera::Error err2 = camera::start(&session2);
		if(err2 != camera::Error::OK) std::cout << "Camera 2: " << camera::get_error_string(err2) << std::endl;

		camera::Error err3 = camera::start(&session3);
		if(err3 != camera::Error::OK) std::cout << "Camera 3: " << camera::get_error_string(err3) << std::endl;
	}
	// Open UDP connection
	{
		network::Error net_err = network::connect(&conn, "192.168.1.1", 45545, 45545);
		if((net_err != network::Error::OK)) {
			std::cerr << "[!]Failed to connect to rover!" << std::endl;
		}
	}



	/*
	 * MAIN LOOP
	 * 1. Process Info Locally
	 * 2. Send out packets
	 */
	while(true) {


		/*
		 * 1. Process Info Locally
		 * - Grabbing frames
		 */
#ifdef TIME
	auto start_all = high_resolution_clock::now();
#endif
		//Grab a frame
		uint8_t* frame_buffer;
		std::size_t frame_size;
		{
			camera::Error err0 = camera::grab_frame(&session0, &frame_buffer, &frame_size);
			if(err0 != camera::Error::OK) std::cout << "Camera 0: " << camera::get_error_string(err0) << std::endl;
		}
		frame_counter ++;
#ifdef DEBUG
		// Print camera buffer info
		std::cout << (void*)frame_buffer << std::endl;
		std::cout << "1" << camera::get_error_string(err1) << std::endl;
#endif
#ifdef TIME
	auto end_grab_frame = high_resolution_clock::now();
	std::cout << 
		"[LOCAL_TIME] Time to GRAB FRAME from session0: " <<
		duration_cast<milliseconds>(end_grab_frame - start_all).count() <<
		"ms" <<
	std::endl;
#endif


		/*
		 * 2. Send out packets
		 * - Send Logging packets
		 * - Create camera messages and send Camera packets
		 */
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
		// THIS IS SAMPLE CODE to send outgoing packets //
		// Log messages
		network::Buffer* log_buffer = network::get_outgoing_buffer();
		char s[] = "[LOG] Hewwo Wowld!";
		network::LogMessage log_message = {static_cast<uint8_t>(strlen(s)), s};
		network::serialize(log_buffer, &log_message);

		network::queue_outgoing(&conn, network::MessageType::LOG, log_buffer);

		// Camera Frames
		/*
		 * This 'for' loop makes sure we create 4 camera streams, one for each camera.
		 * This is TEMPORARY CODE that will be removed once we have a better system
		 *     to organize our objects that will allow us to iterate through
		 *     a collection of camera objects    -yu
		 */
		std::vector<uint8_t*> data_refs; //keep a reference of all our buffers so we can free them
		for (unsigned int i = 0; i < 4; i++) {
			// Calculate how many buffers we will need to send the entire frame
			uint8_t num_buffers = (frame_size/CAMERA_MESSAGE_SIZE) + 1;
			for (unsigned int j = 0; j < num_buffers; j++) {
				network::Buffer* camera_buffer = network::get_outgoing_buffer();

				//This accounts for the last buffer that is not completely divisible
				//by the defined buffer size, by using up the remaining space calculated
				//with modulus    -yu
				uint16_t buffer_size = (j != num_buffers-1)?CAMERA_MESSAGE_SIZE:frame_size%CAMERA_MESSAGE_SIZE;

				network::CameraMessage message = {
					static_cast<uint8_t>(i),              // stream_index
					static_cast<uint16_t>(frame_counter), // frame_index
					static_cast<uint8_t>(j),              // section_index
					num_buffers,                          // section_count
					buffer_size,                          // size
					nullptr                               // data
				};
				message.data = new uint8_t[CAMERA_MESSAGE_SIZE];
				data_refs.push_back(message.data); // keep a ref

				// OPTIMIZE: reuse the same message object to avoid reallocating memory
				memcpy(message.data, frame_buffer + (CAMERA_MESSAGE_SIZE*j), CAMERA_MESSAGE_SIZE);
				network::serialize(camera_buffer, &message);

				network::queue_outgoing(&conn, network::MessageType::CAMERA, camera_buffer);
			}
		}

		network::drain_outgoing(&conn);
		for(auto ref: data_refs) { delete ref; } // free refs
		
		/// Return all buffers
		{
			camera::Error err0 = camera::return_buffer(&session0, frame_buffer);
			if(err0 != camera::Error::OK) std::cout << "Camera 0: " << camera::get_error_string(err0) << std::endl;
		}


#ifdef DEBUG
		std::cout << "2" << camera::get_error_string(err1) << std::endl;
#endif
#ifdef TIME
		auto end_all = high_resolution_clock::now();
		std::cout << 
			"[LOCAL_TIME] Time to CREATE and SEND all Logging and Camera buffers: " <<
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
