#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>

#include <cstring>
#include <string>

#include "network.h"

NetworkManager::NetworkManager(std::string address_string, int port)
{
    receive_timestamp = 0;
    send_timestamp = 0;

    socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd < 0)
    {
        // Socket open failure
        printf("[!] Failed to open socket!\n");
    }

    //TODO: Explain this block
    struct sockaddr_in address;
    memset((char*)&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    inet_aton(address_string.c_str(), &address.sin_addr);
    address.sin_port = htons(port);

    if (bind(socket_fd, (struct sockaddr*) &address, sizeof(address)) < 0)
    {
        // Bind failure
        printf("[!] Failed to bind socket!\n");
    }
}

NetworkManager::~NetworkManager()
{
    close(socket_fd);
}

#define BUFFERVALUE(buff, byte_offset, data_type) *((data_type*) &buff[byte_offset])

void NetworkManager::send_packet(PacketType type, void* packet, size_t packet_length, std::string address, int port)
{
    uint8_t buffer[HEADER_LENGTH + packet_length];
    
    BUFFERVALUE(buffer, 0, uint16_t) = htons(CURRENT_ROVER_PROTOCOL_VERSION);
    BUFFERVALUE(buffer, 2, uint8_t) = (uint8_t) type;
    BUFFERVALUE(buffer, 3, uint16_t) = htons(send_timestamp);

    // Do our own overflow, since its undefined for C++.
    if (send_timestamp == UINT16_MAX)
    {
        send_timestamp = 0;
    }
    else
    {
        send_timestamp++;
    }

    switch (type)
    {
        case PacketType::PING:
            BUFFERVALUE(buffer, HEADER_LENGTH, uint8_t) = (uint8_t) ((PacketPing*) packet)->ping_direction;
            break;
        case PacketType::CONTROL:
            BUFFERVALUE(buffer, HEADER_LENGTH, uint8_t) = (uint8_t) ((PacketControl*) packet)->movement_state;
            break;
        case PacketType::CAMERA:
            PacketCamera* camera = (PacketCamera*) packet;
            BUFFERVALUE(buffer, HEADER_LENGTH, uint16_t) = camera->size;
            memcpy(&buffer[HEADER_LENGTH + 2], camera->data, (size_t) camera->size);
            break;
    }

    // TODO: Explain this block
    struct sockaddr_in send_addr;
    memset((char*)&send_addr, 0, sizeof(send_addr));
    send_addr.sin_family = AF_INET;
    send_addr.sin_port = htons(port);
    inet_aton(address.c_str(), &send_addr.sin_addr);

    if (sendto(socket_fd, buffer, MAXIMUM_PACKET_LENGTH, 0, (struct sockaddr*) &send_addr, sizeof(send_addr)) < 0)
    {
        // Send failure
        printf("[!] Failed to send packet!\n");
    }
}

void NetworkManager::set_packet_handler(PacketType pt, PacketHandler h)
{
    packet_handlers[(uint8_t) pt] = h;
}

void NetworkManager::poll()
{
    uint8_t buffer[MAXIMUM_PACKET_LENGTH];
    struct sockaddr src_addr;
    socklen_t src_addr_len;

    ssize_t res;

    while (1)
    {
        src_addr_len = sizeof(src_addr);

        res = recvfrom(socket_fd, buffer, MAXIMUM_PACKET_LENGTH, MSG_DONTWAIT, &src_addr, &src_addr_len);
        if (res == -1)
        {
            // Two options here: either its because no packets were around, or there's an actual error...
            if (errno == EAGAIN)
            {
                // printf("> Nothing to read.\n");
                break;
            }
            else
            {
                // Handle failure
                printf("[!] Failed to receive packet!\n");
                break;
            }
        }

        // Get sender information
        struct sockaddr_in src_addr_in = (struct sockaddr_in) src_addr;
        int port = (int) ntohs(src_addr_in.sin_port);
        std::string address(inet_ntoa(src_addr_in.sin_addr));

        // We have to convert non-byte values from network order (Big Endian) to host order.

        uint16_t version = ntohs(BUFFERVALUE(buffer, 0, uint16_t));
        uint8_t type = BUFFERVALUE(buffer, 2, uint8_t);
        uint16_t timestamp = ntohs(BUFFERVALUE(buffer, 3, uint8_t);

        // Handle the version.
        if (version != CURRENT_ROVER_PROTOCOL_VERSION)
        {
            printf("[!] Ignoring packet with incorrect version!\n");
            continue;
        }

        // Handle the timestamp.
        if (timestamp == 0)
        {
            receive_timestamp = 0;
        }
        else
        {
            if (timestamp <= receive_timestamp)
            {
                printf("[!] Ignoring out-of-order packet!\n");
                continue;
            }
        }

        receive_timestamp = timestamp;

        switch ((PacketType) type)
        {
            case PacketType::PING: {
                PacketPing packet = { (PingDirection) buffer[HEADER_LENGTH] };
                packet_handlers[type](*this, &packet, address, port);
                break;
            }
            case PacketType::CONTROL: {
                PacketControl packet = { (MovementState) buffer[HEADER_LENGTH] };
                packet_handlers[type](*this, &packet, address, port);
                break;
            }
            default: {
                // TODO: Handle unknown packet
                break;
            }
        }
    }
}
