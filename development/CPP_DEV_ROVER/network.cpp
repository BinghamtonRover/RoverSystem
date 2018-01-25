#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "network.h"

void NetworkHandler::NetworkHandler(std::string address, int port) {
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

void NetworkHandler::~NetworkHandler() {

}

void NetworkHandler::send_packet(Packet* p) {

}

void NetworkHandler::set_packet_handler(PacketType pt, PacketHandler h) {

}