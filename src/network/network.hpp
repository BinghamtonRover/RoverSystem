#ifndef NETWORK_HPP
#define NETWORK_HPP

#include <cstdint>

#include <queue>

#include "arena.hpp"

/*
    This library provides network support with a custom protocol over UDP.
    For more information about the protocol, the spec can be found in the docs folder of the repository.

    Here is an example program:

    ```
    int main() {
        // Allocate a connection.
        // This struct holds all information about a single socket. To use multiple sockets, create multiple
   connections. network::Connection conn;

        network::Error error;
        // Connect to the network library running on 192.168.1.1:45545. Open local port 45545.
        if ((error = network::connect(&conn, "192.168.1.1", 45545, 45545)) != network::Error::OK) {
            fprintf(stderr, "[!] Failed to connect to rover!\n");
            return 1;
        }

        // The network library is designed to be invoked in a main loop.
        while (true) {
            // First, process incoming packets in the kernel packet queue.
            network::poll_incoming(&conn);

            // Allocate a message struct to receive each message that just arrived.
            network::Message message;

            // dequeue_incoming returns false when there are no more messages to be processed.
            while (network::dequeue_incoming(&conn, &message)) {

                // Check what type of message this is and handle it accordingly.
                switch (message.type) {

                    case network::MessageType::HEARTBEAT:

                        // This message doesn't have a body, so it doesn't need to be read.
                        // To send a message, first get a buffer. Don't allocate these yourself!
                        network::Buffer* outgoing = network::get_outgoing_buffer();

                        // This puts the message on a queue. The message will be sent on the next drain_outgoing call.
                        // The HEARTBEAT packet in this example doesn't have any data, so we didn't write anything to
   the buffer. network::queue_outgoing(&conn, network::MessageType::HEARTBEAT, outgoing); break;

                    case network::MessageType::ECHO:

                        // This message type has some data in it, assume its a string variable 'str'.
                        // So we can create a struct which represents one and call deserialize on the buffer that came
   with the incoming message! network::EchoMessage echo; network::deserialize(message.buffer, &echo);

                        // In this example, our ECHO reverses the characters it receives in str.
                        // ... Reverse the characters in echo.str ...

                        // To send, first get an outgoing buffer.
                        network::Buffer* outgoing = network::get_outgoing_buffer();

                        // Serialize our reply, which includes the reversed string.
                        network::serialize(outgoing, &echo);

                        // Place the message on a queue to be sent later.
                        network::queue_outgoing(&conn, network::MessageType::ECHO, outgoing);
                        break;
                }

                // Make sure to always return the message buffer when you are finished with it!
                network::return_incoming_buffer(message.buffer);
            }

            // This must be called once per main loop. It sends all queued messages.
            network::drain_outgoing(&conn);
        }
    }
    ```

    This library follows the natural programmatic symmetry of read data, operate on the data, and output the data. As
   such, your main loop networking code will contain some helpful symmetry:

        At the top of your loop, a call to poll_incoming will read packets off the wire...
        ... and at the bottom, drain_outgoing will write packets to the wire.

        Between poll_incoming and drain_outgoing, calls to dequeue_incoming read messages out of the received packets...
        ... and queue_outgoing puts messages into packets.

        A call to dequeue_incoming provides a Buffer, which must be returned with return_incoming_buffer...
        ... and a Buffer can be obtained from get_outgoing_buffer and is returned with queue_outgoing.

    This library is NOT complete. Right now, it can sends and receives fixed-length messages and handles acknowledgement
   and order-sensitivity. It is missing the following:
        - Session/connection status: there is currently no concept of a real connection.
        - Network statistics: we are not recording bandwidth, packet loss, etc.
        - Repeated messages: the library currently does not discard repeat incoming messages.
        - Variable length messages: currently, each message must be the size that its type mandates.
        - A way to get a descriptive string from an error code.
*/

namespace network
{

const int PROTOCOL_VERSION = 2;

// Maximum size of any single UDP packet sent by this protocol.
const unsigned int BUFFER_SIZE = 65500;

const unsigned int PACKET_HEADER_SIZE = 8;
const unsigned int MESSAGE_HEADER_SIZE = 3;

// Number of messages held at any time in the ack table.
const int ACK_TABLE_LEN = 32;

//
// Buffer Definition
//

struct Buffer
{
    uint8_t buffer[BUFFER_SIZE];
    int size;
    int idx;

    bool incoming;
};

void init_buffer(Buffer *buffer, bool incoming);

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
        4. Add an entry to message_type_info.
*/

enum class MessageType : uint8_t
{
    HEARTBEAT,
    MOVEMENT,
    CAMERA,
    LOG,

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

struct MessageTypeInfo
{
    bool order, ack;
};

constexpr MessageTypeInfo message_type_info[] = {[(uint8_t)MessageType::HEARTBEAT] = {false, true},
                                                 [(uint8_t)MessageType::MOVEMENT] = {true, false},
                                                 [(uint8_t)MessageType::CAMERA] = {false, false},
                                                 [(uint8_t)MessageType::LOG] = {false, true}};

//
// Core API Definitions
//

enum class Error
{
    OK,
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

// Entry for the AckTable below.
struct AckTableEntry
{
    bool not_yet_acked = false;
    Message message;
};

// Holds ack messages until we have confirmation of their arrival.
struct AckTable
{
    int next_index = 0;

    AckTableEntry table[ACK_TABLE_LEN] = {};
};

struct Connection
{
    int socket_fd;

    std::queue<Message> incoming_queue;
    std::queue<Message> outgoing_queue;

    // Ack information for the other side's messages.
    uint16_t last_ack_idx = 0;
    uint32_t ack_diff = 0;

    // Table to keep information about our ack'ed messages.
    AckTable ack_table = {};

    uint16_t next_outgoing_idx = 0;

    uint16_t last_received_idx[(int)MessageType::NUM] = {};

    const char *destination_address;
    int destination_port;
    int local_port;

    // TODO: Statistic information ...
};

/*
    Opens a socket for communication and binds that socket to a port.
    Also sets the destination for outgoing messages.

    Parameters:
        conn: The connection state object. This should be uninitialized.
        destination_address: The destination ip address of the connection.
        destination_port: The destination port of the connection.
        local_port: The local port for the network socket.

    Returns Error::OK on success and a relevant error on failure:
        Error::OPEN_SOCKET: Failed to open a network socket.
        Error::BIND_SOCKET: Failed to bind the opened socket to the given local port.
*/
Error connect(Connection *conn, const char *destination_address, int destination_port, int local_port);

/*
    Queues a message for sending. This also returns ownership of the buffer
    to the library, so it is invalid after this call.

    The message is not sent until a call to `drain_outgoing`.

    Parameters:
        conn: The connection state object. Must be initialized with `connect`.
        type: The type of the message to be sent.
        buffer: A buffer containing the message body. A buffer must be obtained from `get_outgoing_buffer`.
*/

Error reconnect(Connection *conn, const char *destination_address, int destination_port, int local_port);
void queue_outgoing(Connection *conn, MessageType type, Buffer *buffer);

Error check_status(Connection *conn);

/*
    Dequeues (pops) a message from the incoming buffer and places it in the given pointer.

    The buffer returned in message must be returned after use with `return_incoming_buffer`!

    Parameters:
        conn: The connection state object. Must be initialized with `connect`.
        message: The message struct to fill in.

    Returns true if a message was available. `dequeue_incoming` should be called in a loop until
    it returns false, at which point there are no messages. Note that when `dequeue_incoming` returns
    false, no message was read.
*/
bool dequeue_incoming(Connection *conn, Message *message);

/*
    Reads incoming packets and places their messages onto the incoming message queue.

    Parameters:
        conn: The connection state object. Must be initialized with `connect`.

    Returns Error::OK on success and a relevant error on failure:
        Error::READ_PACKET: Failed to read a UDP packet.
        Error::WRONG_VERSION: Received a packet using the wrong protocol version.
*/
Error poll_incoming(Connection *conn);

/*
    Sends the messages queued by `queue_outgoing`.

    Parameters:
        conn: The connection state object. Must be initialized with `connect`.

    Returns Error::OK on success and a relevant error on failure:
        Error::SEND_PACKET: Failed to send a packet.
*/
Error drain_outgoing(Connection *conn);

/*
    Returns a buffer for use with `queue_outgoing`. Once given to `queue_outgoing`, the
    buffer is no longer valid and must not be used.
*/
Buffer *get_outgoing_buffer();

/*
    Hands ownership of a buffer back to the library. This must be called with a buffer
    obtained from `poll_incoming` after its contents are processed.

    Parameters:
        buffer: A buffer obtained from `poll_incoming`.
*/

void return_incoming_buffer(Buffer* buffer);

int get_bandwidth(Buffer* buffer);


} // namespace network

#endif
