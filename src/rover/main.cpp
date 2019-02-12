#include "camera.hpp"
#include "../network/network.hpp"
#include "../shared.hpp"

#include <turbojpeg.h>

#include <sys/ioctl.h>
#include <cstddef>
#include <cstdio>
#include <iostream>
#include <vector>
#include <cstring>
#include <cstdlib>

// GLOBAL CONSTANTS

const int MAX_STREAMS = 4;
const unsigned int CAMERA_WIDTH = 1920;
const unsigned int CAMERA_HEIGHT = 1080;

int main() {
	unsigned int frame_counter = 0;

	// Camera streams
	std::vector<camera::CaptureSession*> streams;

	// Try to open MAX_STREAMS streams.
	for (int i = 0; i < MAX_STREAMS; i++) {
		camera::CaptureSession* cs = new camera::CaptureSession;

		char filename_buffer[13]; // "/dev/video" is 10 chars long, leave 2 for numbers, and one for null terminator.
		sprintf(filename_buffer, "/dev/video%d", i);

		camera::Error err = camera::open(cs, filename_buffer, CAMERA_WIDTH, CAMERA_HEIGHT);
		if (err != camera::Error::OK) {
			delete cs;
			break;
		}

		// Start the camera.
		err = camera::start(cs);
		//initialize the id for the camera
		cs->id = i;
		if (err != camera::Error::OK) {
			camera::close(cs);
			delete cs;
			break;
		}

		// It was opened and started. Add it to the list of streams.
		streams.push_back(cs);
	}

	std::cout << "> Using " << streams.size() << " cameras." << std::endl;

	// UDP connection
	network::Connection conn;

	// Open UDP connection
	{
		network::Error net_err = network::connect(&conn, "127.0.0.1", 45545, 45546);
		if(net_err != network::Error::OK) {
			std::cerr << "[!]Failed to connect to base station!" << std::endl;
		}
	}

	/*
		MAIN LOOP
			1. Process Info Locally
			2. Send out packets
	*/
	fd_set fds;
	FD_ZERO(&fds);
	int ready_camera;
	int max_fd = 0;
	struct timeval tv;
	uint8_t * frame_buffer;
	size_t frame_size;
	tv.tv_sec = camera::SELECT_TIMEOUT;
	tv.tv_usec = 0;
	while(true) {
		/*
			1. Process Info Locally
				- Grabbing frames
		*/
		/*
		for (unsigned int i = 0; i < streams.size(); i++) {
			camera::CaptureSession* cs = streams[i];

			// Grab a frame.
			uint8_t* frame_buffer;
			size_t frame_size;
			{
				camera::Error err = camera::grab_frame(cs, &frame_buffer, &frame_size);
				if(err != camera::Error::OK) {
					std::cout << "Camera 0: " << camera::get_error_string(err) << std::endl;
				}
			}
		  */
			for (unsigned int i = 0; i < streams.size(); i++){
				camera::CaptureSession * cs = streams[i];
				if (cs->fd > max_fd){
					max_fd = cs->fd;
				}
				FD_SET(cs->fd,&fds);
				
			}	
			ready_camera = select(max_fd + 1,&fds,NULL,NULL,&tv);
			if ( ready_camera < 0){
			    std::cout << camera::get_error_string(camera::Error::READ_FRAME) << std::endl;
			}
			if (ready_camera == 0){
				continue;
			}
			camera::CaptureSession * ready_session;
			for (unsigned int i = 0; i < streams.size(); i++){
				if(FD_ISSET(streams[i]->fd,&fds)){
					ready_session = streams[i];
					ready_camera = streams[i]->fd;
					struct v4l2_buffer vbuf;
					memset(&vbuf,0,sizeof(vbuf));

					
					vbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
					vbuf.memory = V4L2_MEMORY_USERPTR;
					if (ioctl(ready_camera, VIDIOC_DQBUF, &vbuf) != 0) {
						std::cout << "Camera " << ready_camera << ": " << camera::get_error_string(camera::Error::READ_FRAME) << std::endl;
					}
					frame_buffer = (uint8_t*) vbuf.m.userptr;
					frame_size = vbuf.bytesused;
					ready_session->last_capture_buffer = vbuf;

					// Decode the frame and encode it again to set our desired quality.
					static uint8_t raw_buffer[CAMERA_WIDTH*CAMERA_HEIGHT*3];

					static tjhandle compressor = tjInitCompress();
					static tjhandle decompressor = tjInitDecompress();

					// Decompress into a raw frame.
					tjDecompress2(decompressor, frame_buffer, frame_size, raw_buffer, CAMERA_WIDTH, 3*CAMERA_WIDTH, CAMERA_HEIGHT, TJPF_RGB, 0);

					// Recompress into jpeg buffer.
					tjCompress2(compressor, raw_buffer, CAMERA_WIDTH, 3*CAMERA_WIDTH, CAMERA_HEIGHT, TJPF_RGB, &frame_buffer, &frame_size, TJSAMP_422, 25, TJFLAG_NOREALLOC);
					/*
						2. Send out packets
							- Create camera messages and send Camera packets
					*/

					// Send the frame.
					// Calculate how many buffers we will need to send the entire frame
					uint8_t num_buffers = (frame_size/CAMERA_MESSAGE_FRAME_DATA_MAX_SIZE) + 1;

					for (unsigned int j = 0; j < num_buffers; j++) {
						network::Buffer* camera_buffer = network::get_outgoing_buffer();

						// This accounts for the last buffer that is not completely divisible
						// by the defined buffer size, by using up the remaining space calculated
						// with modulus    -yu
						uint16_t buffer_size = (j != num_buffers-1) ? CAMERA_MESSAGE_FRAME_DATA_MAX_SIZE : frame_size%CAMERA_MESSAGE_FRAME_DATA_MAX_SIZE;

						network::CameraMessage message = {
							static_cast<uint8_t>(ready_session->id),              						// stream_index
							static_cast<uint16_t>(frame_counter), 						// frame_index
							static_cast<uint8_t>(j),              						// section_index
							num_buffers,                          						// section_count
							buffer_size,                          						// size
							frame_buffer + (CAMERA_MESSAGE_FRAME_DATA_MAX_SIZE*j)       // data
						};

						memcpy(message.data, frame_buffer + (CAMERA_MESSAGE_FRAME_DATA_MAX_SIZE*j), buffer_size);
						network::serialize(camera_buffer, &message);

						network::queue_outgoing(&conn, network::MessageType::CAMERA, camera_buffer);
					}	

					camera::return_buffer(ready_session);
				

				// Increment global (across all streams) frame counter. Should be ok. Should...
				frame_counter++;

				network::poll_incoming(&conn);
				network::Message message;
				// Receive incoming messages
				while(network::dequeue_incoming(&conn, &message)) {
					switch(message.type) {
						case network::MessageType::HEARTBEAT: {
							network::Buffer* outgoing = network::get_outgoing_buffer();
							network::queue_outgoing(&conn, network::MessageType::HEARTBEAT, outgoing);
							break;
						}
						case network::MessageType::MOVEMENT: {
							network::MovementMessage movement;
							network::deserialize(message.buffer, &movement);

							printf("> Current movement: %d, %d\n", movement.left, movement.right);
							break;
						}
						default:
							break;
					}

					network::return_incoming_buffer(message.buffer);
				}

				network::drain_outgoing(&conn);
			    }	
			}
		}

	return 0;
}
