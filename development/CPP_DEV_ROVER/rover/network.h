#include <unordered_map>
#include <cstdint>

#define CURRENT_ROVER_PROTOCOL_VERSION 4
#define HEADER_LENGTH 5

enum class PacketType : uint8_t
{
    PING = 0,
    CONTROL = 1,
    CAMERA = 2
};

enum class MovementState : uint8_t
{
    STOP = 0,
    FORWARD = 1,
    LEFT = 2,
    RIGHT = 3,
    BACKWARD = 4
};

#define PACKET_CONTROL_MAX 1
struct PacketControl
{
    MovementState movement_state;
};

enum class PingDirection : uint8_t
{
    PING = 0,
    PONG = 1
};

#define PACKET_PING_MAX 1
struct PacketPing
{
    PingDirection ping_direction;
};

#define PACKET_CAMERA_MAX 40002
struct PacketCamera
{
    uint16_t size;
    uint8_t* data;
};

// Reference to manager, pointer to packet, sender address, sender port
typedef void (*PacketHandler)(NetworkManager&, void*, std::string, int);

class NetworkManager
{
    private:
        std::unordered_map<uint8_t, PacketHandler> packet_handlers;
        int socket_fd;
        uint16_t receive_timestamp, send_timestamp;

    public:
        NetworkManager(std::string, int);
        ~NetworkManager();

        void send_packet(PacketType, void*, size_t, std::string, int);
        void set_packet_handler(PacketType, PacketHandler);
        void poll();
};