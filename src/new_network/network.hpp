#include <stdint.h>

#include "memory.hpp"

namespace network {

//
// Definitions
//

const int MAX_RAW_PACKET_SIZE = 1500; // Use Ethernet MTU.
// Header:
//     u8  protocol_version
//     u8  message_type
//     u16 message_size
const int HEADER_SIZE = 1 + 1 + 2;
const int MAX_MESSAGE_SIZE = MAX_RAW_PACKET_SIZE - HEADER_SIZE;

const int MAX_IDLE_TIME = 2000; // In ms.
const int MAX_HEARTBEAT_WAIT_TIME = 1000; // In ms.

enum class Error {
    OK,

    NOMORE
};

//
// End Definitions
//

//
// Feed Management
//

struct InFeed {
   int port; 
   int socket_fd;

   MemoryPool memory_pool;
};

struct OutFeed {
    int port;
    int socket_fd;

    MemoryPool memory_pool;
};

Error init(int port, InFeed* out_feed);
Error init(int port, OutFeed* out_feed);

void close(InFeed* feed);
void close(OutFeed* feed);

//
// End Feed Management
//

//
// Buffer Stuff
//

struct Buffer {
    uint16_t index;
    uint16_t size;
    uint16_t cap;

    uint8_t* data;
};

template<typename T>
void deserialize(Buffer* buffer, T* t) {
    t->deserialize(buffer);
}

void serialize(Buffer* buffer, uint8_t value);
void deserialize(Buffer* buffer, uint8_t* value);

void serialize(Buffer* buffer, uint16_t value);
void deserialize(Buffer* buffer, uint16_t* value);

void serialize(Buffer* buffer, int16_t value);
void deserialize(Buffer* buffer, int16_t* value);

void serialize(Buffer* buffer, uint32_t value);
void deserialize(Buffer* buffer, uint32_t* value);

void serialize(Buffer* buffer, int32_t value);
void deserialize(Buffer* buffer, int32_t* value);

void serialize(Buffer* buffer, uint64_t value);
void deserialize(Buffer* buffer, uint64_t* value);

void serialize(Buffer* buffer, int64_t value);
void deserialize(Buffer* buffer, int64_t* value);

Buffer* get_outgoing_buffer(OutFeed* feed);

//
// End Buffer Stuff
//

//
// Message Definitions
//

enum class MessageType {
    HEARTBEAT
};

struct HeartbeatMessage {
    static const auto TYPE = MessageType::HEARTBEAT;

    void serialize(Buffer* buffer) {}
    void deserialize(Buffer* buffer) {}
};

//
// End Message Definitions
//

//
// Sending and Receiving
//

struct IncomingMessage {
    MessageType type;
    Buffer* buffer;
};

Error receive(InFeed* feed, IncomingMessage* out_message);
Error send(OutFeed* feed, MessageType type, Buffer* buffer);

template<typename T>
Error publish(OutFeed* feed, T* thing) {
    auto buffer = get_outgoing_buffer(feed);
    auto message_type = T::TYPE;

    thing->serialize(buffer);

    return send(feed, message_type, buffer);
}

//
// End Sending and Receiving
//

} // namespace network
