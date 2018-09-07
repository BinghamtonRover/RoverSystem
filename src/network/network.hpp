#ifndef NETWORK_HPP
#define NETWORK_HPP

#include <endian.h>
#include <string.h>

#include <queue>

#include "../utils/utils.hpp"
#include "arena.hpp"

#define NETWORK_BUFFER_SIZE 65500

namespace network
{

class IncomingBuffer
{
private:

    u8 buffer[NETWORK_BUFFER_SIZE];
    int buffer_size;
    int buffer_idx;

public:

    void bytes(u8* data, int size)
    {
        memcpy(data, buffer + buffer_idx, size);
        buffer_idx += size;
    }

    inline bool incoming() const
    {
        return true;
    }
};

class OutgoingBuffer
{
private:

    u8 buffer[NETWORK_BUFFER_SIZE];
    int buffer_idx;

public:

    void bytes(u8* data, int size)
    {
        memcpy(buffer + buffer_idx, data, size);
        buffer_idx += size;
    }

    inline bool incoming() const
    {
        return false;
    }
};

//
// Type Serialization Definitions
//

template <typename Buffer>
void serialize(Buffer buffer, u8& v)
{
    buffer.bytes(&v, 1);
}

template <typename Buffer>
void serialize(Buffer buffer, i8& v)
{
    serialize(buffer, (u8&)v);
}

template <typename Buffer>
void serialize(Buffer buffer, u16& v)
{
    if (buffer.incoming())
    {
        v = be16toh(v);
        buffer.bytes((u8*)&v, 2);
    } else
    {
        u16 v1 = htobe16(v);
        buffer.bytes((u8*)&v1, 2);
    }
}

template <typename Buffer>
void serialize(Buffer buffer, i16& v)
{
    serialize(buffer, (u16&)v);
}

template <typename Buffer>
void serialize(Buffer buffer, u32& v)
{
    if (buffer.incoming())
    {
        v = be32toh(v);
        buffer.bytes((u8*)&v, 4);
    } else
    {
        u32 v1 = htobe32(v);
        buffer.bytes((u8*)&v1, 4);
    }
}

template <typename Buffer>
void serialize(Buffer buffer, i32& v)
{
    serialize(buffer, (u32&) v);
}

//
// Message Type Definitions
//

enum class MessageType : u8
{
    HEARTBEAT,
    MOVEMENT,

    NUM
};

struct MovementMessage
{
    i16 left;
    i16 right;

    template <typename Buffer>
    void serialize(Buffer buffer) {
        serialize(buffer, left);
        serialize(buffer, right);
    }
};

struct MessageTypeRequirements
{
    bool order;
    bool ack;
};

const MessageTypeRequirements message_type_requirements[] =
{
    [(u8)MessageType::HEARTBEAT] = {false, true},
    [(u8)MessageType::MOVEMENT]  = {true , false}
};

//
// Manager Definition
//

struct MessageContainer
{
    MessageType type;
    u16 index;
    union {
        IncomingBuffer* incoming_buffer;
        OutgoingBuffer* outgoing_buffer;
    }
}

class Manager
{
private:

    Arena<IncomingBuffer> incoming_arena;
    Arena<OutgoingBuffer> outgoing_arena;

    std::queue<MessageContainer> incoming_queue;
    std::queue<MessageContainer> outgoing_queue;

public:

    Manager();

    OutgoingBuffer* get_outgoing_buffer();
    void queue_message(MessageType type, OutgoingBuffer* buffer);

    bool poll_message(IncomingBuffer** buffer);
};

} // namespace network

#endif