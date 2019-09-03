#include <endian.h>
#include <assert.h>
#include <string.h>

#include "network.hpp"

namespace network {

//
// Serialization Primitives
//

static void check_buffer_write_space(Buffer* buffer, int bytes) {
    assert(buffer->cap - buffer->size >= bytes);
}

static void check_buffer_read_space(Buffer* buffer, int bytes) {
    assert(buffer->size - buffer->index >= bytes);
}

void serialize(Buffer* buffer, uint8_t value) {
    check_buffer_write_space(buffer, 1);
    memcpy(buffer->data + buffer->size, &value, 1);
    buffer->size += 1;
}

void deserialize(Buffer* buffer, uint8_t* value) {
    check_buffer_read_space(buffer, 1);
    memcpy(value, buffer->data + buffer->index, 1);
    buffer->index += 1;
}


void serialize(Buffer* buffer, uint16_t value) {
    check_buffer_write_space(buffer, 2);
    value = htobe16(value);
    memcpy(buffer->data + buffer->size, &value, 2);
    buffer->size += 2;
}

void deserialize(Buffer* buffer, uint16_t* value) {
    check_buffer_read_space(buffer, 2);
    memcpy(value, buffer->data + buffer->index, 2);
    *value = be16toh(*value);
    buffer->index += 2;
}


void serialize(Buffer* buffer, int16_t value) {
    serialize(buffer, *reinterpret_cast<uint16_t*>(&value));
}

void deserialize(Buffer* buffer, int16_t* value) {
    deserialize(buffer, reinterpret_cast<uint16_t*>(value));
}


void serialize(Buffer* buffer, uint32_t value) {
    check_buffer_write_space(buffer, 4);
    value = htobe32(value);
    memcpy(buffer->data + buffer->size, &value, 4);
    buffer->size += 4;
}

void deserialize(Buffer* buffer, uint32_t* value) {
    check_buffer_read_space(buffer, 4);
    memcpy(value, buffer->data + buffer->index, 4);
    *value = be32toh(*value);
    buffer->index += 4;
}


void serialize(Buffer* buffer, int32_t value) {
    serialize(buffer, *reinterpret_cast<uint32_t*>(&value));
}

void deserialize(Buffer* buffer, int32_t* value) {
    deserialize(buffer, reinterpret_cast<uint32_t*>(value));
}


void serialize(Buffer* buffer, uint64_t value) {
    check_buffer_write_space(buffer, 8);
    value = htobe64(value);
    memcpy(buffer->data + buffer->size, &value, 8);
    buffer->size += 8;
}

void deserialize(Buffer* buffer, uint64_t* value) {
    check_buffer_read_space(buffer, 8);
    memcpy(value, buffer->data + buffer->index, 8);
    *value = be64toh(*value);
    buffer->index += 8;
}


void serialize(Buffer* buffer, int64_t value) {
    serialize(buffer, *reinterpret_cast<uint64_t*>(&value));
}

void deserialize(Buffer* buffer, int64_t* value) {
    deserialize(buffer, reinterpret_cast<uint64_t*>(value));
}

//
// End Serialization Primitives
//

//
// Feed Management
//

Error init(int port, InFeed* out_feed) {
    return Error::OK;
}

Error init(int port, OutFeed* out_feed) {
    return Error::OK;
}

void close(InFeed* feed) {
}

void close(OutFeed* feed) {
}

//
// End Feed Management
//

//
// Sending and Receiving
//

Error receive(InFeed* feed, IncomingMessage* out_message) {
    return Error::OK;
}

Error publish(OutFeed* feed, MessageType type, Buffer* buffer) {
    return Error::OK;
}

//
// End Sending and Receiving
//

} // namespace network

