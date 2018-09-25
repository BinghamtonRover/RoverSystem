
#include "network.hpp"

namespace network
{

//
// Primitive serialization functions.
//

void serialize(Buffer* buffer, uint8_t v) {
    VALUE(buffer, buffer->idx, uint8_t) = v;
    buffer->idx += 1;
}

void serialize(Buffer* buffer, int8_t v) {
    serialize(buffer, reinterpret_cast<uint8_t>(v));
}

void serialize(Buffer* buffer, uint16_t v) {
    VALUE(buffer, buffer->idx, uint16_t) = htobe16(v);
    buffer->idx += 2;
}

void serialize(Buffer* buffer, int16_t v) {
    serialize(buffer, reinterpret_cast<uint16_t>(v));
}

void serialize(Buffer* buffer, uint32_t v) {
    VALUE(buffer, buffer->idx, uint32_t) = htobe32(uint32_t);
    buffer->idx += 4;
}

void serialize(Buffer* buffer, int32_t v) {
    serialize(buffer, reinterpret_cast<uint32_t>(v));
}

//
// Primitive deserialization functions.
//

void deserialize(Buffer* buffer, uint8_t* v) {
    *v = VALUE(buffer, buffer->idx, uint8_t);
    buffer->idx += 1;
}

void deserialize(Buffer* buffer, int8_t* v) {
    deserialize(buffer, static_cast<uint8_t*>(v));
}

void deserialize(Buffer* buffer, uint16_t* v) {
    *v = be16toh(VALUE(buffer, buffer->idx, uint16_t));
    buffer->idx += 2;
}

void deserialize(Buffer* buffer, int16_t* v) {
    deserialize(buffer, static_cast<uint16_t*>(v));
}

void deserialize(Buffer* buffer, uint32_t* v) {
    *v = be32toh(VALUE(buffer, buffer->idx, uint32_t));
    buffer->idx += 4;
}

void deserialize(Buffer* buffer, int32_t* v) {
    deserialize(buffer, static_cast<uint32_t*>(v));
}

//
// Message serialization and deserialization functions.
//

void serialize(Buffer* buffer, MovementMessage* message) {
    serialize(buffer, message->left);
    serialize(buffer, message->right);
}

void deserialize(Buffer* buffer, MovementMessage* message) {
    deserialize(buffer, message->left);
    deserialize(buffer, message->right);
}

//
// Core functionality.
//

// A global arena for in and out buffers.
// This is consistent across all connections.
// TODO: This makes the library non-threadsafe! Maybe this should be per-connection.
static Arena<Buffer> buffer_arena;

Error connect(Connection* conn, const char* destination_address, int destination_port, int local_port) {
    // Zero the index information.
    conn->next_outgoing_idx = 0;

    // Note that this technically implies that idx `0` has already been read for every packet type.
    // For now, comparisons against last_received_idx[i] will take this into account and only count
    // a packet as out-of-order if it is strictly less than last_received_idx[i].
    // 
    // This does imply that repeat packets are not dealt with, which could be a problem with
    // ack'ed packets.
    // TODO: Handle duplicate packets?
    memset(conn->last_received_idx, 0, sizeof(uint16_t)*(int)MessageType::NUM);
    
    // Open a socket.
    // We specify AF_INET to use IPv4.
    // We specify SOCK_DGRAM to indicate UDP.
    // This does indeed force UDP, see man IP(7) for details.
    // We supply 0 for the protocol ID, since IP only offers UDP for SOCK_DGRAM.
    conn->socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (conn->socket_fd < 0) {
        return Error::OPEN_SOCKET;
    }

    // Next, we must bind the socket.

    // First, allocate an address struct and clear it.
    // This will be used to set our local socket address.
    // The _in suffix indicates that this struct is used for INET(IPv4) communication.
    struct sockaddr_in bind_address;
    memset(&bind_address, 0, sizeof(bind_address));

    // AF_INET indicates IPv4.
    bind_address.sin_family = AF_INET;

    // Parses and assigns the local bind address.
    // The value "0.0.0.0" indicates "bind on any interface". If we require a more complicated
    // network stack in the future with multiple network interfaces, this would be replaced
    // with an IP address assigned to that interface.
    // TODO: Robustness: this works if we don't care about interfaces.
    inet_aton("0.0.0.0", &bind_address.sin_addr);

    // Set the local port.
    // The bind call below expects this to be in network byte order (big-endian).
    address.sin_port = htobe16(local_port);

    // Now the socket is bound to the address and port we have specified above.
    // This function takes a generic sockaddr struct, but our sockaddr_in can be
    // cast to a sockaddr struct.
    if (bind(conn->socket_fd, (struct sockaddr*) &bind_address, sizeof(bind_address)) < 0) {
        return Error::BIND_SOCKET;
    }

    return Error::OK;
}

void queue_outgoing(Connection* conn, MessageType type, Buffer* buffer) {
    // Create a message and add it to the outgoing queue.
    // Also increment the next outgoing index. Don't worry, this will overflow
    // and wrap as we expect, I promise.
    conn->outgoing_queue.push({ type, conn->next_outgoing_idx++, buffer });

    // Note that the buffer is not returned to the arena here.
    // Yes, I lied in the function header documentation.
    // The buffer will be returned when the message is dequeued and sent.
    // This is because we don't want someone overwriting our buffer in the meantime,
    // which is very possible considering more than one message is likely to be queued.
}

// A convenience function for comparing message indices which takes wraparound into account.
// Adapted from 
// https://gafferongames.com/post/reliability_ordering_and_congestion_avoidance_over_udp#handling-sequence-number-wrap-around.
inline bool out_of_order(uint16_t last, uint16_t next) {
    return  ( ( last > next ) && ( last - next <= 32768 ) ) || 
            ( ( last < next ) && ( next - last  > 32768 ) );
}

Error poll_incoming(Connection* connt) {
    // General strategy: loop and accumulate messages until EAGAIN.
    // Process those messages (update order & ack state).
    // Place those messages on the incoming queue.

    // Allocate a buffer to use for this packet.
    // TODO: Performance! This implies that each message must be copied to its
    // own buffer. If this function is taking a while, look into this!
    Buffer buffer;
    init_buffer(&buffer, true);

    // These two fields are updated every loop by recvfrom.
    // They can start uninitialized.
    // The address of the sender.
    struct sockaddr src_addr;
    // The length of the address of the sender.
    socklen_t src_addr_len;

    while (true) {
        // recvfrom expects us to update this every call.
        src_addr_len = sizeof(src_addr);

        // Try to receive a UDP packet from the socket described by conn->socket_fd.
        // Use buffer to receive the contents, and buffer is of size READ_BUFFER_SIZE.
        // MSG_DONTWAIT makes this call non-blocking.
        // The src_addr and src_addr_len will be filled. For now, we don't use them,
        // but we may need to in the future.
        ssize_t res = recvfrom(conn->socket_fd, buffer.buffer, BUFFER_SIZE, MSG_DONTWAIT, &src_addr, &src_addr_len);
        if (res == -1) {
            // There was an error.
            // Two options: No packets left to read, or a read error.

            if (res == EAGAIN || res == EWOULDBLOCK) {
                // No packets left.
                break;
            }

            // A read error.
            return Error::READ_PACKET;
        }

        // We need to read the header first.
        // See the network spec (/docs/network.md) for explanation.
        uint8_t version;
        uint16_t last_ack_index;
        uint32_t ack_diff;
        uint8_t num_messages;

        deserialize(buffer, &version);
        deserialize(buffer, &last_ack_index);
        deserialize(buffer, &ack_diff);
        deserialize(buffer, &num_messages);

        if (version != PROTOCOL_VERSION) {
            return Error::WRONG_VERSION;
        }

        // TODO: Update ack table.

        for (uint8_t i = 0; i < num_messages; i++) {
            // For each message, read its header first.
            uint16_t index;
            uint8_t type;

            deserialize(buffer, &index);
            deserialize(buffer, &type);

            // TODO: Assert that the type is valid?

            // Grab info about the message type.
            MessageTypeInfo info = message_type_info[type];

            if (info.order) {
                // Check to see if the index is < last_received_idx[type].
                // We need to do this in a way that takes wraparound into account.
                // For instance, 0 < 35565, but message 0 came "after" 35565, so
                // our method should claim that 0 ">" 35565.
                if (out_of_order(conn->last_received_idx[type], index)) {
                    // TODO: Record that we have dropped an out-of-order packet.

                    // Drop it!
                    // We need to advance the packet buffer by the length of the message.
                    buffer.idx += info.size;
                    continue;
                }
            }

            if (info.ack) {
                // We must update our record of the other side's ack messages.

                // Two scenarios:
                //  (a) This message has idx > last_ack_idx;
                //  (b) This message has idx == last_ack_idx, where we do nothing.
                //  (c) This message has idx < last_ack_idx (which is allowed).
                if (index > conn->last_ack_idx) {
                    uint16_t idx_diff = index - conn->last_ack_idx;

                    // We need to shift idx_diff bits off to the left of the ack_diff,
                    // since our new last_ack_idx is idx_diff greater.
                    conn->ack_diff <<= idx_diff;

                    // Update our last ack.
                    conn->last_ack_idx = index;

                    // We must also set the bit for the old last_ack_idx, assuming it didn't
                    // fall off the end of the diff.
                    if (idx_diff <= 32) {
                        // Now we set the bit which represents last_ack_idx, which is
                        // the (index - idx_diff - 1)th bit.
                        // If we shift by 1, then the LSB must be set, so we need to subtract 1
                        // from the idx_diff here.
                        conn->ack_diff |= (1 << (idx_diff - 1));
                    }
                } else if (index < conn->last_ack_idx) {
                    // We simply set the relevant bit!
                    uint16_t idx_diff = conn->last_ack_idx - index;

                    // See above for how this bit is calculated.
                    conn->ack_diff |= (1 << (idx_diff - 1));
                }
            }

            // Grab a buffer for the message.
            Buffer* message_buffer = buffer_arena.alloc();
            init_buffer(message_buffer, true);

            message_buffer->size = info.size;

            // Copy the message from where we are in the packet buffer to the message buffer.
            memcpy(message_buffer->buffer, buffer.buffer + buffer.idx, info.size);

            // Update the index of our packet buffer.
            buffer.idx += info.size;

            // Create and push a Message for this message!
            incoming_queue.push({ type, index, message_buffer });
        }
    }
}

} // namespace network