#include <unordered_map>
#include <stdint.h>

#define CURRENT_ROVER_PROTOCOL_VERSION 3
#define MAXIMUM_PACKET_LENGTH 6 // This includes the header

enum class PacketType : uint8_t {
    PING = 0,
    CONTROL = 1
}

struct Packet {
    uint16_t version;
    PacketType type;
    uint16_t timestamp;

    void* body;
};

enum class MovementState : uint8_t {
    STOP = 0,
    FORWARD = 1,
    LEFT = 2,
    RIGHT = 3
};

struct PacketControl {
    MovementState movement_state;
};

enum class PingDirection : unsigned char {
    PING = 0,
    PONG = 1
}

struct PacketPing {
    PingDirection ping_direction;
}

typedef void (*PacketHandler)(Packet*);

class NetworkManager {
private:
    std::unordered_map<PacketType, PacketHandler> packet_handlers;

    int socket_fd;

public:

    NetworkManager(std::string, int);
    ~NetworkManager();

    void send_packet(Packet*, std::string, int);
    void set_packet_handler(PacketType, PacketHandler);

    void poll();
};