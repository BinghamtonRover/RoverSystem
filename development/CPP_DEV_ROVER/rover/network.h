#include <unordered_map>
#include <cstdint>

#define CURRENT_ROVER_PROTOCOL_VERSION 3
#define MAXIMUM_PACKET_LENGTH 6 // This includes the header

enum class PacketType : uint8_t
{
    PING = 0,
    CONTROL = 1
};

enum class MovementState : uint8_t
{
    STOP = 0,
    FORWARD = 1,
    LEFT = 2,
    RIGHT = 3,
    BACKWARD = 4
};

struct PacketControl
{
    MovementState movement_state;
};

enum class PingDirection : uint8_t
{
    PING = 0,
    PONG = 1
};

struct PacketPing
{
    PingDirection ping_direction;
};

typedef void (*PacketHandler)(void*);

class NetworkManager
{
    private:
        std::unordered_map<uint8_t, PacketHandler> packet_handlers;
        int socket_fd;
        uint16_t receive_timestamp, send_timestamp;

    public:
        NetworkManager(std::string, int);
        ~NetworkManager();

        void send_packet(PacketType, void*, std::string, int);
        void set_packet_handler(PacketType, PacketHandler);
        void poll();
};