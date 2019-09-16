#include <endian.h>
#include <assert.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "network.hpp"

/*
    The network library uses multicast. Here's a good link about the proper Unix multicast procedure:
        http://www.kohala.com/start/mcast.api.txt
*/

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
    memcpy(buffer->data + buffer->index, &value, 1);
    buffer->size += 1;
    buffer->index += 1;
}

void deserialize(Buffer* buffer, uint8_t* value) {
    check_buffer_read_space(buffer, 1);
    memcpy(value, buffer->data + buffer->index, 1);
    buffer->index += 1;
}


void serialize(Buffer* buffer, uint16_t value) {
    check_buffer_write_space(buffer, 2);
    value = htobe16(value);
    memcpy(buffer->data + buffer->index, &value, 2);
    buffer->size += 2;
    buffer->index += 2;
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
    memcpy(buffer->data + buffer->index, &value, 4);
    buffer->size += 4;
    buffer->index += 4;
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
    memcpy(buffer->data + buffer->index, &value, 8);
    buffer->size += 8;
    buffer->index += 8;
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
// Buffer Stuff
//

Buffer get_outgoing_buffer(Feed* feed) {
    auto buffer_bytes = feed->memory_pool.alloc();

    Buffer buffer;
    // Leave room for the header.
    buffer.index = HEADER_SIZE;
    buffer.size = HEADER_SIZE;
    buffer.cap = (uint16_t) feed->memory_pool.element_size;
    buffer.data = buffer_bytes;
    
    return buffer;
}

static Buffer get_incoming_buffer(Feed* feed) {
    auto buffer_bytes = feed->memory_pool.alloc();

    Buffer buffer;
    buffer.index = 0;
    buffer.size = 0;
    buffer.cap = (uint16_t) feed->memory_pool.element_size;
    buffer.data = buffer_bytes;
    
    return buffer;
}

//
// End Buffer Stuff
//

//
// Feed Management
//

Error init_subscriber(const char* group, uint16_t port, Feed* out_feed) {
    int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd == -1) {
        return Error::SOCKET;
    }
    out_feed->socket_fd = sock_fd;

    // Bind to the correct port.
    struct sockaddr_in bind_addr{};
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_port = htobe16(port);
    bind_addr.sin_addr.s_addr = htobe32(INADDR_ANY);

    if (bind(sock_fd, (struct sockaddr*) &bind_addr, sizeof(bind_addr)) == -1) {
        ::close(sock_fd);
        return Error::BIND;
    }

    // Join the multicast group.
    struct ip_mreq mreq;
    mreq.imr_multiaddr.s_addr = inet_addr(group);
    mreq.imr_interface.s_addr = htobe32(INADDR_ANY);
    
    if (setsockopt(sock_fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0) {
        ::close(sock_fd);
        return Error::MULTICAST_JOIN;    
    }

    out_feed->memory_pool.init(MAX_RAW_PACKET_SIZE, 64);

    return Error::OK;
}

Error init_publisher(const char* group, uint16_t port, Feed* out_feed) {
    int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd == -1) {
        return Error::SOCKET;
    }

    out_feed->socket_fd = sock_fd;
    struct sockaddr_in connect_addr{};
    connect_addr.sin_family = AF_INET;
    connect_addr.sin_port = htobe16((short)port);
    connect_addr.sin_addr.s_addr = inet_addr(group);

    if (connect(sock_fd, (struct sockaddr*)&connect_addr, sizeof(connect_addr)) == -1) {
        ::close(sock_fd);
        return Error::CONNECT;
    }

    out_feed->memory_pool.init(MAX_RAW_PACKET_SIZE, 64);

    return Error::OK;
}

void close(Feed* feed) {
    ::close(feed->socket_fd);
    feed->memory_pool.close();
}

//
// End Feed Management
//

//
// Sending and Receiving
//

Error receive(Feed* feed, IncomingMessage* out_message) {
    auto buffer = get_incoming_buffer(feed);

    auto res = recv(feed->socket_fd, buffer.data, buffer.cap, MSG_DONTWAIT);
    buffer.size = (uint16_t) res;

    if (res == -1) {
        feed->memory_pool.free(buffer.data);

        if (errno == EAGAIN && errno == EWOULDBLOCK) {
            return Error::NOMORE;
        }

        return Error::RECEIVE;
    }

    uint8_t version;
    uint8_t message_type;
    uint16_t message_size;

    deserialize(&buffer, &version);
    deserialize(&buffer, &message_type);
    deserialize(&buffer, &message_size);

    if (version != PROTOCOL_VERSION) {
        feed->memory_pool.free(buffer.data);
        return Error::VERSION;
    }

    out_message->type = (MessageType) message_type;
    out_message->buffer = buffer;

    return Error::OK;
}

Error send(Feed* feed, MessageType type, Buffer buffer) {
    assert(buffer.size <= buffer.cap);
    assert(buffer.cap <= MAX_RAW_PACKET_SIZE);

    // Fill header.

    buffer.index = 0;
    // Necessary since serialize calls increment size.
    buffer.size -= HEADER_SIZE;

    // Record the actual message (sans header) size here.
    auto message_size = (uint16_t) buffer.size;

    serialize(&buffer, (uint8_t) PROTOCOL_VERSION);
    serialize(&buffer, (uint8_t) type);
    serialize(&buffer, message_size);

    if (::send(feed->socket_fd, buffer.data, buffer.size, 0) == -1) {
        return Error::SEND;
    }

    // Discard buffer.
    feed->memory_pool.free(buffer.data);

    return Error::OK;
}

//
// End Sending and Receiving
//

} // namespace network

