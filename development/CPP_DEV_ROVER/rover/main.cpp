#include <unistd.h>

#include "network.h"

static void handle_ping(void* void_packet) {
    PacketPing packet = *((PacketPing*) void_packet);

    printf("> Received PING packet with direction %d\n", (uint8_t) packet.ping_direction);
}

static void handle_control(void* void_packet) {
    PacketControl packet = *((PacketControl*) void_packet);

    std::string message;

    switch (packet.movement_state) {
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
}

int main() {
    NetworkManager manager("127.0.0.1", 35353);

    manager.set_packet_handler(PacketType::PING, handle_ping);
    manager.set_packet_handler(PacketType::CONTROL, handle_control);

    for (;;) {
        manager.poll();

        sleep(1);
    }
}