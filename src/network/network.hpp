#ifndef NETWORK_HPP
#define NETWORK_HPP

#include <endian.h>
#include <string.h>

#include <queue>

#include "arena.hpp" 

namespace network
{

const int PROTOCOL_VERSION = 1;

const unsigned int BUFFER_SIZE = 65500;

const unsigned int PACKET_HEADER_SIZE = 8;
const unsigned int MESSAGE_HEADER_SIZE = 3;

// Number of messages held at any time in the ack table.
const int ACK_TABLE_LEN = 32;

//
// Buffer Definition
//

struct Buffer {
    uint8_t buffer[BUFFER_SIZE];
    int size;
    int idx;

    bool incoming;
};

void init_buffer(Buffer* buffer, bool incoming) {
    buffer->size = 0;
    buffer->idx = 0;
    buffer->incoming = incoming;
}

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
#define VALUE(thing, idx, type) *((type*)&(thing[idx]))

//
// Type Serialization Definitions
//

void serialize(Buffer* buffer, uint8_t v);
void serialize(Buffer* buffer, int8_t v);
void serialize(Buffer* buffer, uint16_t v);
void serialize(Buffer* buffer, int16_t v);
void serialize(Buffer* buffer, uint32_t v);
void serialize(Buffer* buffer, int32_t v);

//
// Type Deserialization Funcitons
//

void deserialize(Buffer* buffer, uint8_t* v);
void deserialize(Buffer* buffer, int8_t* v);
void deserialize(Buffer* buffer, uint16_t* v);
void deserialize(Buffer* buffer, int16_t* v);
void deserialize(Buffer* buffer, uint32_t* v);
void deserialize(Buffer* buffer, int32_t* v);

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

enum class MessageType : uint8_t {
    HEARTBEAT,
    MOVEMENT,

    NUM
};


struct MovementMessage {
    int16_t left;
    int16_t right;
};

void serialize(Buffer* buffer, MovementMessage* message);
void deserialize(Buffer* buffer, MovementMessage* message);

struct MessageTypeInfo {
    bool order;
    bool ack;
    uint16_t size; // In bytes.
};

constexpr MessageTypeinfo message_type_info[] = {
    [(uint8_t)MessageType::HEARTBEAT] = {false, true, 0},
    [(uint8_t)MessageType::MOVEMENT]  = {true , false, 4}
};

//
// Core API Definitions
//

enum class Error {
    OK,

    OPEN_SOCKET,
    BIND_SOCKET,
    READ_PACKET,
    WRONG_VERSION,
    SEND_PACKET
}

struct Message {
    MessageType type;
    uint16_t index;
    Buffer* buffer;
}

// Entry for the AckTable below.
struct AckTableEntry {
    bool not_yet_acked = false;
    Message message;
}

// Holds ack messages until we have confirmation of their arrival.
struct AckTable {
    int next_index = 0;

    ackTableEntry table[ACK_TABLE_LEN] = {};
};

struct Connection {
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

    const char* destination_address;
    int destination_port;
    int local_port;

    // TODO: Statistic information ...
}

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
Error connect(Connection* conn, const char* destination_address, int destination_port, int local_port);

/*
    Queues a message for sending. This also returns ownership of the buffer
    to the library, so it is invalid after this call.

    The message is not sent until a call to `drain_outgoing`.

    Parameters:
        conn: The connection state object. Must be initialized with `connect`.
        type: The type of the message to be sent.
        buffer: A buffer containing the message body. A buffer must be obtained from `get_outgoing_buffer`.
*/
void queue_outgoing(Connection* conn, MessageType type, Buffer* buffer);

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
bool dequeue_incoming(Connection* conn, Message* message);

/*
    Reads incoming packets and places their messages onto the incoming message queue.

    Parameters:
        conn: The connection state object. Must be initialized with `connect`.

    Returns Error::OK on success and a relevant error on failure:
        Error::READ_PACKET: Failed to read a UDP packet.
        Error::WRONG_VERSION: Received a packet using the wrong protocol version.
*/
Error poll_incoming(Connection* conn);

/*
    Sends the messages queued by `queue_outgoing`.

    Parameters:
        conn: The connection state object. Must be initialized with `connect`.
    
    Returns Error::OK on success and a relevant error on failure:
        Error::SEND_PACKET: Failed to send a packet.
*/
Error drain_outgoing(Connection* conn);

/*
    Returns a buffer for use with `queue_outgoing`. Once given to `queue_outgoing`, the
    buffer is no longer valid and must not be used.
*/
Buffer* get_outgoing_buffer();

/*
    Hands ownership of a buffer back to the library. This must be called with a buffer
    obtained from `poll_incoming` after its contents are processed.

    Parameters:
        buffer: A buffer obtained from `poll_incoming`.
*/
void return_incoming_buffer(Buffer* buffer);


} // namespace network

#endif