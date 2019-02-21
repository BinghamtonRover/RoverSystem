#include <cerrno>
#include <cstring>

#include <arpa/inet.h>
#include <endian.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include "network.hpp"

namespace network
{

void init_buffer(Buffer *buffer, bool incoming)
{
    buffer->size = 0;
    buffer->idx = 0;
    buffer->incoming = incoming;
}

//
// Primitive serialization functions.
//

template <> void serialize(Buffer *buffer, uint8_t v)
{
    // See network.hpp for info about the VALUE macro.
    VALUE(buffer->buffer, buffer->idx, uint8_t) = v;

    // Update the buffer size and index.
    buffer->idx += 1;
    buffer->size += 1;
}

template <> void serialize(Buffer *buffer, int8_t v)
{
    // Interpret the bytes of v as if it were unsigned.
    serialize(buffer, *reinterpret_cast<uint8_t *>(&v));
}

template <> void serialize(Buffer *buffer, uint16_t v)
{
    // Convert the value to big endian form before sending.
    VALUE(buffer->buffer, buffer->idx, uint16_t) = htobe16(v);
    buffer->idx += 2;
    buffer->size += 2;
}

template <> void serialize(Buffer *buffer, int16_t v)
{
    serialize(buffer, *reinterpret_cast<uint16_t *>(&v));
}

template <> void serialize(Buffer *buffer, uint32_t v)
{
    VALUE(buffer->buffer, buffer->idx, uint32_t) = htobe32(v);
    buffer->idx += 4;
    buffer->size += 4;
}

template <> void serialize(Buffer *buffer, int32_t v)
{
    serialize(buffer, *reinterpret_cast<uint32_t *>(&v));
}

//
// Primitive deserialization functions.
//

template <> void deserialize(Buffer *buffer, uint8_t *v)
{
    *v = VALUE(buffer->buffer, buffer->idx, uint8_t);
    buffer->idx += 1;
}

template <> void deserialize(Buffer *buffer, int8_t *v)
{
    deserialize(buffer, reinterpret_cast<uint8_t *>(v));
}

template <> void deserialize(Buffer *buffer, uint16_t *v)
{
    *v = be16toh(VALUE(buffer->buffer, buffer->idx, uint16_t));
    buffer->idx += 2;
}

template <> void deserialize(Buffer *buffer, int16_t *v)
{
    deserialize(buffer, reinterpret_cast<uint16_t *>(v));
}

template <> void deserialize(Buffer *buffer, uint32_t *v)
{
    *v = be32toh(VALUE(buffer->buffer, buffer->idx, uint32_t));
    buffer->idx += 4;
}

template <> void deserialize(Buffer *buffer, int32_t *v)
{
    deserialize(buffer, reinterpret_cast<uint32_t *>(v));
}

//
// Message serialization and deserialization functions.
//

template <> void serialize(Buffer *buffer, MovementMessage *message)
{
    serialize(buffer, message->left);
    serialize(buffer, message->right);
}

template <> void deserialize(Buffer *buffer, MovementMessage *message)
{
    deserialize(buffer, &message->left);
    deserialize(buffer, &message->right);
}

template <> void serialize(Buffer *buffer, CameraMessage *message)
{
    serialize(buffer, message->stream_index);
    serialize(buffer, message->frame_index);
    serialize(buffer, message->section_index);
    serialize(buffer, message->section_count);
    serialize(buffer, message->size);

    memcpy(buffer->buffer + buffer->idx, message->data, message->size);
    buffer->idx += message->size;
    buffer->size += message->size;
}

// See the note in network.hpp regarding memory management.
template <> void deserialize(Buffer *buffer, CameraMessage *message)
{
    deserialize(buffer, &(message->stream_index));
    deserialize(buffer, &(message->frame_index));
    deserialize(buffer, &(message->section_index));
    deserialize(buffer, &(message->section_count));
    deserialize(buffer, &(message->size));

    if (message->data == nullptr) {
        message->data = new uint8_t[message->size];
    }

    memcpy(message->data, buffer->buffer + buffer->idx, message->size);
    buffer->idx += message->size;
}

template <> void serialize(Buffer *buffer, LogMessage *message)
{
    serialize(buffer, message->size);

    memcpy(buffer->buffer + buffer->idx, message->log_string, message->size);
    buffer->idx += message->size;
    buffer->size += message->size;
}

template <> void deserialize(Buffer *buffer, LogMessage *message)
{
    deserialize(buffer, &(message->size));

    memcpy(message->log_string, buffer->buffer + buffer->idx, message->size);
    buffer->idx += message->size;
}

//
// Core functionality.
//

// A global arena for in and out buffers.
// This is consistent across all connections.
// TODO: This makes the library non-threadsafe! Maybe this should be per-connection.
static Arena<Buffer> buffer_arena(10);

Error connect(Connection *conn, const char *destination_address, int destination_port, int local_port)
{
    // Set the destination information.
    conn->destination_address = destination_address;
    conn->destination_port = destination_port;

    // And set our port!
    conn->local_port = local_port;

    // Note that this technically implies that idx `0` has already been read for every packet type.
    // For now, comparisons against last_received_idx[i] will take this into account and only count
    // a packet as out-of-order if it is strictly less than last_received_idx[i].
    //
    // This does imply that repeat packets are not dealt with, which could be a problem with
    // ack'ed packets.
    // TODO: Handle duplicate packets?
    memset(conn->last_received_idx, 0, sizeof(uint16_t) * (int)MessageType::NUM);

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

    // Specify that we can bind to any interface.
    bind_address.sin_addr.s_addr = INADDR_ANY;

    // Set the local port.
    // The bind call below expects this to be in network byte order (big-endian).
    bind_address.sin_port = htobe16(local_port);

    // Now the socket is bound to the address and port we have specified above.
    // This function takes a generic sockaddr struct, but our sockaddr_in can be
    // cast to a sockaddr struct.
    if (bind(conn->socket_fd, (struct sockaddr *)&bind_address, sizeof(bind_address)) < 0) {
        return Error::BIND_SOCKET;
    }

    return Error::OK;
}

void queue_outgoing(Connection *conn, MessageType type, Buffer *buffer)
{
    // Create a message and add it to the outgoing queue.
    // Also increment the next outgoing index. Don't worry, this will overflow
    // and wrap as we expect, I promise.
    conn->outgoing_queue.push({type, conn->next_outgoing_idx++, buffer});

    // Note that the buffer is not returned to the arena here.
    // Yes, I lied in the function header documentation.
    // The buffer will be returned when the message is dequeued and sent.
    // This is because we don't want someone overwriting our buffer in the meantime,
    // which is very possible considering more than one message is likely to be queued.
}

// A convenience function for comparing message indices which takes wraparound into account.
// Adapted from
// https://gafferongames.com/post/reliability_ordering_and_congestion_avoidance_over_udp#handling-sequence-number-wrap-around.
inline bool out_of_order(uint16_t last, uint16_t next)
{
    return ((last > next) && (last - next <= 32768)) || ((last < next) && (next - last > 32768));
}

Error poll_incoming(Connection *conn)
{
    // General strategy: loop and accumulate messages until EAGAIN.
    // Process those messages (update order & ack state).
    // Place those messages on the incoming queue.

    // Allocate a buffer to use for this packet.
    // TODO: Performance! This implies that each message must be copied to its
    // own buffer. If this function is taking a while, look into this!
    Buffer buffer;

    // These two fields are updated every loop by recvfrom.
    // They can start uninitialized.
    // The address of the sender.
    struct sockaddr src_addr;
    // The length of the address of the sender.
    socklen_t src_addr_len;

    while (true) {
        // Initialize buffer every time.
        init_buffer(&buffer, true);

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

            if (errno == EAGAIN || errno == EWOULDBLOCK) {
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

        deserialize(&buffer, &version);
        deserialize(&buffer, &last_ack_index);
        deserialize(&buffer, &ack_diff);
        deserialize(&buffer, &num_messages);

        if (version != PROTOCOL_VERSION) {
            return Error::WRONG_VERSION;
        }

        // Update the ack table based on what we got back.
        for (uint16_t i = 0; i < 32; i++) {
            // For each entry in the ack diff...

            // Calculate the message index.
            // We shift 1 by i to get the ith bit, and then and that with
            // the diff to see if that bit is set.
            if (ack_diff & (1 << i)) {
                // The "+1" accounts for the fact that bit 0 represents a difference of 1.
                uint16_t idx = last_ack_index - i + 1;

                // If that index is in the ack table, get rid of that message.
                // TODO: Performance! This is O(N) and we can do it in O(1).
                // However, I don't think this will be a bottleneck, simply
                // because we will always be IO-bound.
                for (int j = 0; j < ACK_TABLE_LEN; j++) {
                    AckTableEntry *entry = conn->ack_table.table + j;
                    if (entry->not_yet_acked && entry->message.index == idx) {
                        // We found our message and it needs to be removed from the table.
                        // Return its buffer and flip not_yet_acked.
                        buffer_arena.free(entry->message.buffer);
                        entry->not_yet_acked = false;
                        break;
                    }
                }
            }
        }

        for (uint8_t i = 0; i < num_messages; i++) {
            // For each message, read its header first.
            uint16_t index;
            uint8_t type;
            uint16_t size;

            deserialize(&buffer, &index);
            deserialize(&buffer, &type);
            deserialize(&buffer, &size);

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
                    buffer.idx += size;
                    continue;
                }

                // Update last received.
                conn->last_received_idx[type] = index;
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
                        // The "- 1" accounts for the "1st" bit actually being bit 0.
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
            Buffer *message_buffer = buffer_arena.alloc();
            init_buffer(message_buffer, true);

            message_buffer->size = size;

            // Copy the message from where we are in the packet buffer to the message buffer.
            memcpy(message_buffer->buffer, buffer.buffer + buffer.idx, size);

            // Update the index of our packet buffer.
            buffer.idx += size;

            // Create and push a Message for this message!
            conn->incoming_queue.push({(MessageType)type, index, message_buffer});
        }
    }

    return Error::OK;
}

Error drain_outgoing(Connection *conn)
{
    // Take all the messages off of the queue.
    // Try to put them in the next packet to be sent.
    // If there is no space, put it on another queue.
    // Packets are sent until there are no messages left.

    std::queue<Message> alternate_queue;

    while (!conn->outgoing_queue.empty()) {
        // Collect messages.

        // Allocate a buffer for the outgoing packet.
        // Make sure it is labeled as outgoing.
        Buffer packet_buffer;
        init_buffer(&packet_buffer, false);

        // We need to account for space that the packet header occupies.
        // But that needs to be filled out after we add the packets.
        // So we will leave space for that now, and fill it later.
        packet_buffer.size += PACKET_HEADER_SIZE;
        packet_buffer.idx += PACKET_HEADER_SIZE;

        // We will need to count how many messages we have.
        uint8_t num_messages = 0;

        // Check if we are about to make num_messages overflow.
        // This probably won't ever happen, but its ok to be safe.
        while (!conn->outgoing_queue.empty() && num_messages < UINT8_MAX) {
            // Get a message. This does not remove it from the queue.
            Message m = conn->outgoing_queue.front();
            // This removes it from the queue.
            conn->outgoing_queue.pop();

            // Get information about that message type.
            MessageTypeInfo mti = message_type_info[(int)m.type];

            // Check if it can fit. If not, put it on alternate_queue.
            // Since it needs a header, factor that size in as well.
            if (m.buffer->size + MESSAGE_HEADER_SIZE > BUFFER_SIZE - packet_buffer.size) {
                alternate_queue.push(m);
                continue;
            }

            // Put the header in first.
            uint16_t message_index = m.index;
            uint8_t message_type = (uint8_t)m.type;
            uint16_t message_size = m.buffer->size;
            serialize(&packet_buffer, message_index);
            serialize(&packet_buffer, message_type);
            serialize(&packet_buffer, message_size);

            // Include it in the current packet.
            // Note we use the size as set by the buffer, not the packet size.
            // This is because packet size is actually the maximum size of the packet.
            // This means that if a message doesn't use all of its alotted space, then
            // there will be some garbage following it in the packet.
            // It is up to message types to add a field which indicates the length of
            // any content of variable length.
            // TODO: Do we want to zero out the packet_buffer memory first?
            memcpy(packet_buffer.buffer + packet_buffer.idx, m.buffer->buffer, m.buffer->size);

            // Advance the packet buffer by the size of the buffer size.
            packet_buffer.size += m.buffer->size;
            packet_buffer.idx += m.buffer->size;

            if (mti.ack) {
                // Add this message to the ack table. Don't free its buffer, because
                // we might need to resend it.
                conn->ack_table.table[conn->ack_table.next_index] = {true, m};
                // Make sure the next index wraps around.
                conn->ack_table.next_index = (conn->ack_table.next_index + 1) % ACK_TABLE_LEN;
            } else {
                // We don't need this message anymore, so we can return the buffer.
                buffer_arena.free(m.buffer);
            }

            num_messages++;
        }

        // We need to fill the packet header.
        uint8_t version = PROTOCOL_VERSION;
        uint16_t last_ack_index = conn->last_ack_idx;
        uint32_t ack_diff = conn->ack_diff;
        // uint8_t num_messages was defined above.

        // "cheat" by forcing serialize to put the packet header at the beginning.
        packet_buffer.size -= PACKET_HEADER_SIZE;
        packet_buffer.idx = 0;
        serialize(&packet_buffer, version);
        serialize(&packet_buffer, last_ack_index);
        serialize(&packet_buffer, ack_diff);
        serialize(&packet_buffer, num_messages);

        // We messed with the packet buffer index and didn't reset it, but its okay
        // since we never use it from here on out.
        // The size was displaced by PACKET_HEADER_SIZE worth of writing, so that
        // is still accurate.

        // Now we send the packet.

        // Create an address structure to hold our destination address.
        struct sockaddr_in send_addr;
        // Initialize it to zero.
        memset(&send_addr, 0, sizeof(send_addr));
        // Specify IPv4.
        send_addr.sin_family = AF_INET;
        // Specify the port.
        send_addr.sin_port = htobe16(conn->destination_port);
        // Specify the remote address.
        inet_aton(conn->destination_address, &send_addr.sin_addr);

        // Send the packet!
        // This sends the packet using our socket, using info about our destination as provided by send_addr.
        // sendto returns a negative value on failure.
        if (sendto(conn->socket_fd, packet_buffer.buffer, packet_buffer.size, 0, (struct sockaddr *)&send_addr,
                   sizeof(send_addr)) < 0) {
            return Error::SEND_PACKET;
        }

        // Rotate the queues.
        while (!alternate_queue.empty()) {
            conn->outgoing_queue.push(alternate_queue.front());
            alternate_queue.pop();
        }
    }

    return Error::OK;
}

bool dequeue_incoming(Connection *conn, Message *message)
{
    // Just take the message off the top of the queue, if one exists.

    if (conn->incoming_queue.empty())
        return false;

    *message = conn->incoming_queue.front();
    conn->incoming_queue.pop();

    return true;
}

Buffer *get_outgoing_buffer()
{
    // Grab a buffer and initialize it, setting the direction to outgoing.
    Buffer *b = buffer_arena.alloc();
    init_buffer(b, false);

    return b;
}

void return_incoming_buffer(Buffer *buffer)
{
    buffer_arena.free(buffer);
}

} // namespace network
