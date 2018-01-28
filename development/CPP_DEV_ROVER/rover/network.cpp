#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>

#include "network.h"

NetworkManager::NetworkManager(std::string address, int port) {
    socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd < 0) {
        // TODO: Handle failure
    }

    struct sockaddr_in address;
    address.sin_family = AF_INET;
    inet_aton(address.c_str(), &address.sin_addr);
    address.sin_port = htons(port);

    if (bind(socket_fd, (struct sockaddr*) &address, sizeof(address)) < 0) {
        // TODO: Handle failure
    }
}

NetworkManager::~NetworkManager() {

}

void NetworkManager::send_packet(Packet* p, std::string address, int port) {

}

void NetworkManager::set_packet_handler(PacketType pt, PacketHandler h) {

}

void NetworkManager::poll() {
    uint8_t buffer[MAXIMUM_PACKET_LENGTH];
    struct sockaddr src_addr;
    socklen_t src_addr_len;

    ssize_t res;

    Packet packet;

    while (1) {
        src_addr_len = sizeof(src_addr);

        res = recvfrom(socket_fd, buffer, MAXIMUM_PACKET_LENGTH, MSG_DONTWAIT, &src_addr, &src_addr_len);
        if (res == -1) {
            // Two options here: either its because no packets were around, or there's an actual error...
            if (errno == EAGAIN) {
                break;
            } else {
                // TODO: Handle failure
                break;
            }
        }

        // We have to convert non-byte values from network order (Big Endian) to host order.

        packet.version = ntohs(*((uint16_t*) &buffer[0]));
        packet.type = (PacketType) buffer[2];
        packet.timestamp = ntohs(*((uint16_t*) &buffer[3]));
    }
}