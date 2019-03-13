#ifndef NETWORK_HPP
#define NETWORK_HPP

#include <cstdint>

#include "arena.hpp"

namespace network
{

const int PROTOCOL_VERSION = 5;

// Maximum size of any single UDP packet sent by this protocol.
const unsigned int BUFFER_SIZE = 65500;

const unsigned int HEADER_SIZE = 4;

const int BANDWIDTH_SAMPLE_INTERVAL = 1000;

//
// Buffer Definition
//

struct Buffer
{
    int size;
    int idx;
    uint8_t buffer[HEADER_SIZE + BUFFER_SIZE];
};

void init_buffer(Buffer *buffer);

/*
    This macro gives us the ability to:
        (a) Put a value of an arbitrary type within a byte buffer at an arbitrary location;
        (b) Interpret bytes at an arbitrary location of a byte buffer as an arbitrary type.
    Parameters:
        thing: the byte buffer. Always a uint8_t* for our purposes.
        idx: the byte index into the buffer.
        type: the type being inserted or removed.
    Examples:
        (a) Placing a 32-bit integer into a byte buffer at byte position 7:
            VALUE(buffer, 7, int32_t) = -12344454;
        (b) Interpreting two bytes at byte buffer position 3 as an unsigned 16-bit integer:
            uint16_t my_int = VALUE(buffer, 3, uint16_t);
*/
#define VALUE(thing, idx, type) *((type *)&(thing[idx]))

// Handles serialization of all types.
// All definitions are in network.cpp.
// If something you want to send isn't overloaded, do it there.
template <typename T> void serialize(Buffer *buffer, T t);

// Handles deserialization of all types.
// All definitions are in network.cpp.
// If something you want to receive isn't overloaded, do it there.
template <typename T> void deserialize(Buffer *buffer, T *t);

//
// Message Type Definitions
//

/*
    To add a message type:
        1. Add a value for it to MessageType;
        2. Add a struct to contain its deserialized values;
        3. Define serialize and deserialize overloads here and implement them in network.cpp;
*/

enum class MessageType : uint8_t
{
    HEARTBEAT,
    MOVEMENT,
    CAMERA,
    LOG,
	LIDAR,
	LOCATION,

    NUM
};

struct MovementMessage
{
    int16_t left, right;
};

// When deserializing a CameraMessage, providing data == nullptr will cause a new buffer to be allocated.
// If data != nullptr, the frame data will be written to the memory to which data points. In this case,
// be sure that the size of that block is at least CAMERA_MESSAGE_FRAME_DATA_MAX_SIZE bytes!
// In either case, be sure to free the memory with delete[] once you are finished with that buffer!
struct CameraMessage
{
    uint8_t stream_index;
    uint16_t frame_index;
    uint8_t section_index;
    uint8_t section_count;
    uint16_t size;

    uint8_t *data;
};

struct LogMessage
{
    uint8_t size;

    char *log_string;
};

const int NUM_LIDAR_POINTS = 271;

struct LidarMessage {
	uint16_t points[NUM_LIDAR_POINTS];
};

struct LocationMessage {
	float x;
	float y;
	float z;
	float pitch;
	float yaw;
	float roll;
};

//
// Core API Definitions
//

enum class Error
{
    OK,
	NOMORE,

    DISCONNECT,
    OPEN_SOCKET,
    BIND_SOCKET,
    READ_PACKET,
    WRONG_VERSION,
    SEND_PACKET
};

struct Message
{
    MessageType type;
    uint16_t index;
    Buffer *buffer;
};

struct Connection
{
    int socket_fd;

    const char *destination_address;
    int destination_port;
    int local_port;

	double last_bandwidth;
	int total_bytes;
	unsigned int last_bandwidth_update_time;
};

Error connect(Connection *conn, int local_port, const char *destination_address, int destination_port);

Error send(Connection* conn, MessageType type, Buffer* buffer);

Error poll(Connection* conn, Message* message);

Buffer *get_outgoing_buffer();

void return_incoming_buffer(Buffer *buffer);

double update_bandwidth(Connection* conn, unsigned int time);

} // namespace network

#endif
