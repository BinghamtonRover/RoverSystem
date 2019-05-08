#include <cerrno>
#include <cstring>

#include <arpa/inet.h>
#include <endian.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "network.hpp"

namespace network
{

void init_buffer(Buffer *buffer)
{
    buffer->size = 0;
    buffer->idx = HEADER_SIZE;
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

template <> void serialize(Buffer *buffer, float v)
{
	VALUE(buffer->buffer, buffer->idx, float) = v;
	buffer->idx += 4;
	buffer->size += 4;
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

template <> void deserialize(Buffer *buffer, float* v)
{
	*v = VALUE(buffer->buffer, buffer->idx, float);
	buffer->idx += 4;
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
    deserialize(buffer, &(message->left));
    deserialize(buffer, &(message->right));
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

template <> void serialize(Buffer *buffer, LidarMessage *message) {
	for (int i = 0; i < NUM_LIDAR_POINTS; i++) {
		serialize(buffer, message->points[i]);
	}
}

template <> void deserialize(Buffer *buffer, LidarMessage *message) {
	for (int i = 0; i < NUM_LIDAR_POINTS; i++) {
		deserialize(buffer, &(message->points[i]));
	}
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

template <> void serialize(Buffer *buffer, LocationMessage *message)
{
	serialize(buffer, message->x);
	serialize(buffer, message->y);
	serialize(buffer, message->z);
	serialize(buffer, message->pitch);
	serialize(buffer, message->yaw);
	serialize(buffer, message->roll);
}

template <> void deserialize(Buffer *buffer, LocationMessage *message)
{
	deserialize(buffer, &(message->x));
	deserialize(buffer, &(message->y));
	deserialize(buffer, &(message->z));
	deserialize(buffer, &(message->pitch));
	deserialize(buffer, &(message->yaw));
	deserialize(buffer, &(message->roll));
}

template<>
void serialize(Buffer* buffer, SensorMessage* message){
	serialize(buffer, message->moisture);
	serialize(buffer, message->pressure);
	serialize(buffer, message->altitude);
	serialize(buffer, message->temperature);
}

template<>
void deserialize(Buffer* buffer, SensorMessage* message){
	deserialize(buffer, &(message->moisture));
	deserialize(buffer, &(message->pressure));
	deserialize(buffer, &(message->altitude));
	deserialize(buffer, &(message->temperature));
}


//
// Core functionality.
//

// A global arena for in and out buffers.
// This is consistent across all connections.
// TODO: This makes the library non-threadsafe! Maybe this should be per-connection.
static Arena<Buffer> buffer_arena(10);

Error connect(Connection *conn, int local_port, const char *destination_address, int destination_port) {
    // Set the destination information.
    conn->destination_address = destination_address;
    conn->destination_port = destination_port;

    // And set our port!
    conn->local_port = local_port;

	conn->last_bandwidth = 0;
	conn->total_bytes = 0;
	conn->last_bandwidth_update_time = 0;

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

Error poll(Connection* conn, Message* message) {
	Buffer* buffer = buffer_arena.alloc();
	buffer->idx = 0;
	buffer->size = 0;

    // These two fields are updated every loop by recvfrom.
    // They can start uninitialized.
    // The address of the sender.
    struct sockaddr src_addr;
    // The length of the address of the sender.
    socklen_t src_addr_len;

	// recvfrom expects us to update this every call.
	src_addr_len = sizeof(src_addr);

	// Try to receive a UDP packet from the socket described by conn->socket_fd.
	// Use buffer to receive the contents, and buffer is of size READ_BUFFER_SIZE.
	// MSG_DONTWAIT makes this call non-blocking.
	// The src_addr and src_addr_len will be filled. For now, we don't use them,
	// but we may need to in the future.
	ssize_t res = recvfrom(conn->socket_fd, buffer->buffer, BUFFER_SIZE + HEADER_SIZE, MSG_DONTWAIT, &src_addr, &src_addr_len);
	if (res == -1) {
		// There was an error.
		// Two options: No packets left to read, or a read error.

		if (errno == EAGAIN || errno == EWOULDBLOCK) {
			// No packets left.
			buffer_arena.free(buffer);
			return Error::NOMORE;
		}

		// A read error.
		buffer_arena.free(buffer);
		return Error::READ_PACKET;
	}

	conn->total_bytes += res;

	// We need to read the header first.
	uint8_t version;
	uint8_t type;
	uint16_t size;

	deserialize(buffer, &version);
	deserialize(buffer, &type);
	deserialize(buffer, &size);

	if (version != PROTOCOL_VERSION) {
		buffer_arena.free(buffer);
		return Error::WRONG_VERSION;
	}

	// Make sure the buffer properties are set correctly.
	buffer->idx = HEADER_SIZE;
	buffer->size = size;

	message->buffer = buffer;
	message->type = (MessageType)type;

    return Error::OK;
}

Error send(Connection* conn, MessageType type, Buffer* buffer) {
	uint8_t version = (uint8_t) PROTOCOL_VERSION;
	uint8_t type_enc = (uint8_t) type;
	uint16_t size = buffer->size;

	// Make sure the header part of the buffer is filled properly.
	buffer->idx = 0;

	serialize(buffer, version);
	serialize(buffer, type_enc);
	serialize(buffer, size);

	// Now buffer->idx == HEADER_SIZE.
	// buffer->size is now size + HEADER_SIZE.

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
	if (sendto(conn->socket_fd, buffer->buffer, buffer->size, 0, (struct sockaddr *)&send_addr, sizeof(send_addr)) < 0) {
		buffer_arena.free(buffer);
		return Error::SEND_PACKET;
	}

	conn->total_bytes += buffer->size;

	buffer_arena.free(buffer);
    return Error::OK;
}

Buffer *get_outgoing_buffer()
{
    // Grab a buffer and initialize it, setting the direction to outgoing.
    Buffer *b = buffer_arena.alloc();
    init_buffer(b);

    return b;
}

void return_incoming_buffer(Buffer *buffer)
{
    buffer_arena.free(buffer);
}

double update_bandwidth(Connection* conn, unsigned int time) {
	if (time - conn->last_bandwidth_update_time >= BANDWIDTH_SAMPLE_INTERVAL) {
		conn->last_bandwidth_update_time = time;

		int total_bytes = conn->total_bytes;
		conn->total_bytes = 0;

		conn->last_bandwidth = 0.001 * (total_bytes / (double)BANDWIDTH_SAMPLE_INTERVAL);

		return conn->last_bandwidth;
	}

	return conn->last_bandwidth;
}

} // namespace network
