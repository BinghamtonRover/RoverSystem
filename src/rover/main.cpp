#include "../network/network.hpp"
#include "../shared.hpp"
#include "camera.hpp"
#include "suspension.hpp"
#include "lidar.hpp"

#include <turbojpeg.h>

#include <sl/Camera.hpp>

#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <vector>
#include <chrono>

// GLOBAL CONSTANTS

const int MAX_STREAMS = 2;
const unsigned int CAMERA_WIDTH = 1280;
const unsigned int CAMERA_HEIGHT = 720;

const int LIDAR_SEND_INTERVAL = 1000 / 3;

// Save the start time so we can use get_ticks.
std::chrono::high_resolution_clock::time_point start_time;

unsigned int get_ticks()
{
    auto now = std::chrono::high_resolution_clock::now();

    return (unsigned int)std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
}

struct Config
{
	int local_port;

	char remote_address[16];
	int remote_port;

	char suspension_serial_id[500];
};

Config load_config(const char* filename) {
	Config config;

	FILE* file = fopen(filename, "r");
	if (!file) {
		fprintf(stderr, "[!] Failed to find config file!\n");
		exit(1);
	}

	// First line: local_port remote_address remote_port
	fscanf(file, "%d %s %d\n", &config.local_port, config.remote_address, &config.remote_port);

	// Second line: USB id for the suspension controller.
	fscanf(file, "%s\n", config.suspension_serial_id);

	fclose(file);

	return config;
}

int main()
{
	start_time = std::chrono::high_resolution_clock::now();

	Config config = load_config("res/r_config.txt");

	if (suspension::init(config.suspension_serial_id) != suspension::Error::OK) {
		fprintf(stderr, "[!] Failed to initialize the suspension!\n");
		return 1;
	}

	if (lidar::start("192.168.1.21") != lidar::Error::OK) {
		fprintf(stderr, "[!] Failed to init field LIDAR!\n");
		return 1;
	}

	std::vector<long> lidar_points;

    unsigned int frame_counter = 0;

    // Camera streams
    std::vector<camera::CaptureSession *> streams;

    // Try to open MAX_STREAMS streams.
    for (int i = 0; i < MAX_STREAMS; i++) {
        camera::CaptureSession *cs = new camera::CaptureSession;

		char name_filename_buffer[100];
		sprintf(name_filename_buffer, "/sys/class/video4linux/video%d/name", i);

		FILE* name_file = fopen(name_filename_buffer, "r");
		if (!name_file) continue;

		fscanf(name_file, "%s\n", name_filename_buffer);
		printf("> Found camera with name %s\n", name_filename_buffer);

		fclose(name_file);

		if (strcmp("ZED", name_filename_buffer) == 0) continue;

        char filename_buffer[13]; // "/dev/video" is 10 chars long, leave 2 for numbers, and one for null terminator.
        sprintf(filename_buffer, "/dev/video%d", i);

        camera::Error err = camera::open(cs, filename_buffer, CAMERA_WIDTH, CAMERA_HEIGHT);
        if (err != camera::Error::OK) {
            delete cs;
            break;
        }

        // Start the camera.
        err = camera::start(cs);
        if (err != camera::Error::OK) {
            camera::close(cs);
            delete cs;
			continue;
        }

        // It was opened and started. Add it to the list of streams.
        streams.push_back(cs);
    }

    std::cout << "> Using " << streams.size() << " cameras." << std::endl;

	// Open ZED camera.

	sl::Camera zed;

	sl::InitParameters params;
	params.camera_resolution = sl::RESOLUTION_HD720;
	params.camera_fps = 20;

	auto zed_res = zed.open(params);
	if (zed_res != sl::SUCCESS) {
		fprintf(stderr, "[!] Failed to open ZED camera! Error: %s\n", sl::toString(zed_res).get());
		// return 1;
	}

    // UDP connection
    network::Connection conn;

    // Open UDP connection
    network::Error net_err = network::connect(&conn, config.local_port, config.remote_address, config.remote_port);
    if (net_err != network::Error::OK) {
        std::cerr << "[!]Failed to connect to base station!" << std::endl;
    }

	auto last_lidar_send_time = get_ticks();

    while (true) {
        for (size_t i = 0; i < streams.size(); i++) {
            camera::CaptureSession *cs = streams[i];

            // Grab a frame.
            uint8_t *frame_buffer;
            size_t frame_size;
            {
                camera::Error err = camera::grab_frame(cs, &frame_buffer, &frame_size);
                if (err != camera::Error::OK) {
                    std::cout << "Camera 0: " << camera::get_error_string(err) << std::endl;
                }
            }

            // Decode the frame and encode it again to set our desired quality.
            static uint8_t raw_buffer[CAMERA_WIDTH * CAMERA_HEIGHT * 3];

            static tjhandle compressor = tjInitCompress();
            static tjhandle decompressor = tjInitDecompress();

            // Decompress into a raw frame.
            tjDecompress2(decompressor, frame_buffer, frame_size, raw_buffer, CAMERA_WIDTH, 3 * CAMERA_WIDTH,
                          CAMERA_HEIGHT, TJPF_RGB, 0);

            // Recompress into jpeg buffer.
            tjCompress2(compressor, raw_buffer, CAMERA_WIDTH, 3 * CAMERA_WIDTH, CAMERA_HEIGHT, TJPF_RGB, &frame_buffer,
                        &frame_size, TJSAMP_420, 25, TJFLAG_NOREALLOC);

            /*
                    2. Send out packets
                            - Create camera messages and send Camera packets
            */

            // Send the frame.
            // Calculate how many buffers we will need to send the entire frame
            uint8_t num_buffers = (frame_size / CAMERA_MESSAGE_FRAME_DATA_MAX_SIZE) + 1;

            for (uint8_t j = 0; j < num_buffers; j++) {
                network::Buffer *camera_buffer = network::get_outgoing_buffer();

                // This accounts for the last buffer that is not completely divisible
                // by the defined buffer size, by using up the remaining space calculated
                // with modulus    -yu
                uint16_t buffer_size = (j != num_buffers - 1) ? CAMERA_MESSAGE_FRAME_DATA_MAX_SIZE
                                                              : frame_size % CAMERA_MESSAGE_FRAME_DATA_MAX_SIZE;

                network::CameraMessage message = {
                    static_cast<uint8_t>(i),                                // stream_index
                    static_cast<uint16_t>(frame_counter),                   // frame_index
                    static_cast<uint8_t>(j),                                // section_index
                    num_buffers,                                            // section_count
                    buffer_size,                                            // size
                    frame_buffer + (CAMERA_MESSAGE_FRAME_DATA_MAX_SIZE * j) // data
                };

                // memcpy(message.data, frame_buffer + (CAMERA_MESSAGE_FRAME_DATA_MAX_SIZE*j), buffer_size);
                network::serialize(camera_buffer, &message);

                network::send(&conn, network::MessageType::CAMERA, camera_buffer);
            }

            camera::return_buffer(cs);
        }
#if 1
		sl::Mat zed_image;
		if (zed.grab() == sl::SUCCESS) {
			zed.retrieveImage(zed_image, sl::VIEW_LEFT);

			static tjhandle compressor = tjInitCompress();

			static unsigned long jpeg_size = tjBufSize(1280, 720, TJSAMP_444);
			static uint8_t* jpeg_buffer = (uint8_t*) malloc(jpeg_size);

			auto tj_err = tjCompress2(compressor, zed_image.getPtr<unsigned char>(), 1280, 
									zed_image.getStepBytes(), 720, TJPF_BGRA, 
									(unsigned char**)&jpeg_buffer, &jpeg_size, TJSAMP_444, 40, TJFLAG_NOREALLOC);

/*
			auto tj_err = tjCompressFromYUV(compressor, zed_image.getPtr<unsigned char>(), 1280,
											zed_image.getStepBytes(), 720, TJSAMP_444, &jpeg_buffer,
											&jpeg_size, 40, TJFLAG_NOREALLOC);
*/
			if (tj_err != 0) {
				fprintf(stderr, "[!] tjCompress failed: %s\n", tjGetErrorStr2(compressor));
			}

            // Send the frame.
            // Calculate how many buffers we will need to send the entire frame
            uint8_t num_buffers = (jpeg_size / CAMERA_MESSAGE_FRAME_DATA_MAX_SIZE) + 1;

            for (uint8_t j = 0; j < num_buffers; j++) {
                network::Buffer *camera_buffer = network::get_outgoing_buffer();

                // This accounts for the last buffer that is not completely divisible
                // by the defined buffer size, by using up the remaining space calculated
                // with modulus    -yu
                uint16_t buffer_size = (j != num_buffers - 1) ? CAMERA_MESSAGE_FRAME_DATA_MAX_SIZE
                                                              : jpeg_size % CAMERA_MESSAGE_FRAME_DATA_MAX_SIZE;

                network::CameraMessage message = {
                    static_cast<uint8_t>(streams.size()),                                // stream_index
                    static_cast<uint16_t>(frame_counter),                   // frame_index
                    static_cast<uint8_t>(j),                                // section_index
                    num_buffers,                                            // section_count
                    buffer_size,                                            // size
                    jpeg_buffer + (CAMERA_MESSAGE_FRAME_DATA_MAX_SIZE * j) // data
                };

                // memcpy(message.data, frame_buffer + (CAMERA_MESSAGE_FRAME_DATA_MAX_SIZE*j), buffer_size);
                network::serialize(camera_buffer, &message);

                network::send(&conn, network::MessageType::CAMERA, camera_buffer);
            }
		}
#endif
        // Increment global (across all streams) frame counter. Should be ok. Should...
        frame_counter++;

		// LIDAR stuff.

		if (get_ticks() - last_lidar_send_time >= LIDAR_SEND_INTERVAL) {
			lidar_points.clear();
			if (lidar::scan(lidar_points) != lidar::Error::OK) {
				fprintf(stderr, "[!] Failed to read LIDAR points!\n");
			}

			network::Buffer* buffer = network::get_outgoing_buffer();

			network::LidarMessage message;
			for (int i = 0; i < network::NUM_LIDAR_POINTS; i++) {
				message.points[i] = (uint16_t) lidar_points[i];
			}

			network::serialize(buffer, &message);
			network::send(&conn, network::MessageType::LIDAR, buffer);
		}

        // Receive incoming messages
        network::Message message;
        while (true) {
			network::Error neterr = network::poll(&conn, &message);
			if (neterr != network::Error::OK) {
				if (neterr == network::Error::NOMORE) {
					break;
				}

				break;
			}

            switch (message.type) {
                case network::MessageType::HEARTBEAT: {
                    network::Buffer *outgoing = network::get_outgoing_buffer();
                    network::send(&conn, network::MessageType::HEARTBEAT, outgoing);
                    break;
                }
                case network::MessageType::MOVEMENT: {
                    network::MovementMessage movement;
                    network::deserialize(message.buffer, &movement);

					printf("Got movement with %d, %d\n", movement.left, movement.right);

					suspension::Direction left_direction = movement.left < 0 ? suspension::BACKWARD : suspension::FORWARD;
					suspension::Direction right_direction = movement.right < 0 ? suspension::BACKWARD : suspension::FORWARD; 

					uint8_t left_speed = movement.left < 0 ? (uint8_t) ((-movement.left)) : (uint8_t) (movement.left);
					uint8_t right_speed = movement.right < 0 ? (uint8_t) ((-movement.right)) : (uint8_t) (movement.right);

					suspension::update(suspension::LEFT, left_direction, left_speed);
					suspension::update(suspension::RIGHT, right_direction, right_speed);

                    break;
                }
                default:
                    break;
            }

            network::return_incoming_buffer(message.buffer);
        }
    }
}
