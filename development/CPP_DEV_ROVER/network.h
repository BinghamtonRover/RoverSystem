#include <unordered_map>

#define CURRENT_ROVER_PROTOCOL_VERSION 1

enum class PacketType : unsigned char {
    RESERVED = 0,
    CONTROL = 1
}

struct Packet {
    unsigned short version;
    PacketType type;
    unsigned short timestamp;

    void* body;
};

enum class MovementState : unsigned char {
    STOP = 0,
    FORWARD = 1,
    LEFT = 2,
    RIGHT = 3
};

struct PacketControl {
    MovementState movement_state;
};

typedef void (*PacketHandler)(Packet*);

class NetworkManager {
private:
    std::unordered_map<PacketType, PacketHandler> packet_handlers;

    int socket_fd;

public:

    NetworkManager();
    ~NetworkManager();

    void send_packet(Packet*);
    void set_packet_handler(PacketType, PacketHandler);

    void poll();
};