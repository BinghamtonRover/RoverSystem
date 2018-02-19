#include <cstdlib>
#include <string>
#include <vector>
#include <unistd.h>

#include <iostream>

#include "network.h"
#include "camera.h"

// Camera dimentions.
const int CAMERA_WIDTH = 640;
const int CAMERA_HEIGHT = 360;

using network::PacketHeartbeat;
using network::PacketControl;
using network::PacketCamera;

static void handle_heartbeat(network::Manager& manager, PacketHeartbeat* packet, std::string address, int port)
{
    if (packet->direction == PacketHeartbeat::Direction::PING) {
        printf("> Received ping... responding!\n");
        
        PacketHeartbeat response;
        response.direction = PacketHeartbeat::Direction::PONG;

        manager.send_packet(&response, address, port);
    } else {
        printf("> Our ping was answered by a PONG!\n");
    }
}

static PacketControl::MovementState lastState = PacketControl::MovementState::STOP;

static void handle_control(network::Manager& manager, PacketControl* packet, std::string address, int port) {
    // We do not use these.
    (void)manager;
    (void)address;
    (void)port;

    if (packet->movement_state == lastState)
    {
        return;
    }

    std::string message;

    switch (packet->movement_state)
    {
        case PacketControl::MovementState::STOP:
            message = "stop";
            break;
        case PacketControl::MovementState::FORWARD:
            message = "move forward";
            break;
        case PacketControl::MovementState::LEFT:
            message = "turn left";
            break;
        case PacketControl::MovementState::RIGHT:
            message = "turn right";
            break;
        case PacketControl::MovementState::BACKWARD:
            message = "move backward";
            break;
        default:
            message = "?";
            break;
    }

    printf("> Received CONTROL packet: %s\n", message.c_str());

    lastState = packet->movement_state;
}

uint64_t millisecond_time() {
    struct timespec  ts;
    clock_gettime(CLOCK_REALTIME, &ts);

    return (ts.tv_sec) * 1000 + (ts.tv_nsec) / 1000000;
}

int main(int argc, char** argv)
{

    if (argc != 5)
    {
        printf("[!] Usage: rover <base station address> <base station port> <bind port> <camera device>\n");
        return 1;
    }

    std::string base_station_address(argv[1]);
    int base_station_port = atoi(argv[2]);

    // Initialize the packet readers and writers.
    network::register_packet_functions();

    // Bind to our listening port.
    network::Manager manager("0.0.0.0", atoi(argv[3]));

    // Register packet handlers.
    network::PacketTypeHeartbeat.handler = handle_heartbeat;
    network::PacketTypeControl.handler = handle_control;

    // Set up camera feed.
    camera::CaptureSession camera(CAMERA_WIDTH, CAMERA_HEIGHT);

    if (!camera.open(std::string(argv[4]))) return 1;
    if (!camera.check_capabilities()) return 1;
    if (!camera.init_buffers()) return 1;
    if (!camera.start_stream()) return 1;

    std::cout << "IMAGE SIZE " << camera.image_size << std::endl;
    
    // Create buffer for image data.
    uint8_t* frame_buffer_back = (uint8_t*) malloc(camera.image_size);

    // For cycles per second tracking.
    uint64_t last_time = millisecond_time();
    uint64_t cycles = 0;

    while (1)
    {
        manager.poll();

		uint8_t* frame_buffer = frame_buffer_back;
        
        size_t frame_size = camera.grab_frame(frame_buffer_back);
        if (frame_size == 0) {
            std::cerr << "[!] Failed to grab frame!" << std::endl;
            continue;
        }

        // Create the packets needed.
        int num_packets = (frame_size + (network::CAMERA_PACKET_FRAME_DATA_MAX_SIZE - 1)) / network::CAMERA_PACKET_FRAME_DATA_MAX_SIZE;

        // Send all but the last packet.
        for (int i = 0; i < num_packets - 1; i++) {
            PacketCamera camera_packet;
            camera_packet.section_index = (uint8_t) i;
            camera_packet.section_count = (uint8_t) num_packets;
            camera_packet.size = (uint16_t) network::CAMERA_PACKET_FRAME_DATA_MAX_SIZE;
            camera_packet.data = frame_buffer;

			frame_buffer += network::CAMERA_PACKET_FRAME_DATA_MAX_SIZE;

			// This is for now... java is too slow! We need to get packet size down.
			usleep(10 * 1000);

            manager.send_packet(&camera_packet, base_station_address, base_station_port);            
        }

        // Send the last packet.
        PacketCamera camera_packet;
        camera_packet.section_index = (uint8_t) (num_packets - 1);
        camera_packet.section_count = (uint8_t) num_packets;
        camera_packet.size = (uint16_t) (frame_size % network::CAMERA_PACKET_FRAME_DATA_MAX_SIZE);
        camera_packet.data = frame_buffer;

        manager.send_packet(&camera_packet, base_station_address, base_station_port);

		// Manually increment timestamp
	// Do our own overflow, since its undefined for C++.
	if (manager.send_timestamp == UINT16_MAX)
		manager.send_timestamp = 0;
	else
		manager.send_timestamp++;

        if (cycles % 30 == 0)
            std::cout << "> " << ((float) cycles / (millisecond_time() - last_time)*1000.0) << " cycles per second at millisecond mark " << (millisecond_time() - last_time) << std::endl;
        cycles++;
    }

    free(frame_buffer_back);
}
