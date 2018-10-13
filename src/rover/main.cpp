#include "camera.hpp"
#include <stddef.h>
#include <iostream>
#include "../network/network.hpp"

int main() {
	/// Initialization of Rover objects
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
		//std::cout << "1" << camera::get_error_string(err1) << std::endl;
		err1 = camera::return_buffer(&session1, frame_buffer);
		//std::cout << "2" << camera::get_error_string(err1) << std::endl;

	}
	/// Destruct Rover objects
	// Close the camera session
	camera::close(&session0);
	camera::close(&session1);
	camera::close(&session2);
	camera::close(&session3);
	return 0;
}
