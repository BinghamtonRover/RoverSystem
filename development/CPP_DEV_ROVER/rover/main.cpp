#include <cstdlib>
#include <string>
#include <vector>
#include <unistd.h>

#include <opencv2/opencv.hpp>

#include "network.h"
#include "camera.h"

// A number between 0 and 100 representing the amount of JPEG compression.
// 0 is a blob of pixels and 100 is uncompressed.
#define JPEG_QUALITY 30

static void handle_ping(NetworkManager& manager, void* void_packet, std::string address, int port)
{
    PacketPing packet = *((PacketPing*) void_packet);

    if (packet.ping_direction == PingDirection::PING) {
        printf("> Received ping... responding!\n");
        
        PacketPing response;
        response.ping_direction = PingDirection::PONG;

        manager.send_packet(PacketType::PING, &response, PACKET_PING_MAX, address, port);
    } else {
        printf("> Our ping was answered by a PONG!\n");
    }
}

static MovementState lastState = MovementState::STOP;

static void handle_control(NetworkManager& manager, void* void_packet, std::string address, int port)
{
    // We do not use these.
    (void)manager;
    (void)address;
    (void)port;
    
    PacketControl packet = *((PacketControl*) void_packet);

    if (packet.movement_state == lastState)
    {
        return;
    }

    std::string message;

    switch (packet.movement_state)
    {
        case MovementState::STOP:
            message = "stop";
            break;
        case MovementState::FORWARD:
            message = "move forward";
            break;
        case MovementState::LEFT:
            message = "turn left";
            break;
        case MovementState::RIGHT:
            message = "turn right";
            break;
        case MovementState::BACKWARD:
            message = "move backward";
            break;
        default:
            message = "?";
            break;
    }

    printf("> Received CONTROL packet: %s\n", message.c_str());

    lastState = packet.movement_state;
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

    NetworkManager manager("0.0.0.0", atoi(argv[3]));

    manager.set_packet_handler(PacketType::PING, handle_ping);
    manager.set_packet_handler(PacketType::CONTROL, handle_control);

    // Set up camera feed.
    CaptureSession camera;

    if (!camera.open(std::string(argv[4]))) return 1;
    if (!camera.check_capabilities()) return 1;
    if (!camera.init_buffers()) return 1;
    if (!camera.start_stream()) return 1;
    
    // Create OpenCV matrix for image data.
    cv::Mat image_mat(camera.height, camera.width, CV_8UC3);

    // Create a vector with JPEG quality data.
    // This is for OpenCV's encoding function.
    std::vector<int> encode_options;
    encode_options.push_back(CV_IMWRITE_JPEG_QUALITY);
    encode_options.push_back(JPEG_QUALITY);

    // Create a buffer for JPEG output.
    std::vector<unsigned char> jpeg_buffer;

    while (1)
    {
        manager.poll();
        
        if (!camera.grab_frame(image_mat)) break; // For now... this will be more robust eventually!

        // Encode the image.
        cv::imencode(".jpg", image_mat, jpeg_buffer, encode_options);

        // Create a packet.
        PacketCamera camera_packet;
        camera_packet.size = (uint16_t) jpeg_buffer.size();
        camera_packet.data = (uint8_t*) jpeg_buffer.data();

        // Send the packet.
        manager.send_packet(PacketType::CAMERA, &camera_packet, camera_packet.size + 2, base_station_address, base_station_port);
    }
}
