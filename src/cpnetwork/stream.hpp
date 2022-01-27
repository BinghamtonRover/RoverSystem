/*
	Support for one-way data streams using UDP. Intended for video streaming
	Frames and metadata are sent using vectored IO to reduce copies
*/

#pragma once

#include <cstdint>
#include <vector>
#include <mutex>
#include <shared_mutex>
#include <functional>
#include <stdexcept>
#include <memory>
#include <boost/asio.hpp>

namespace net {

// Stream object may be more useful in the future if we
// want to support named streams and camera UUID
struct StreamMetadata {
	uint8_t frame_index = 0;
};

// Simple stream sender for transmitting frames associated with a stream index.
// StreamSender is not designed for concurrent sends
class StreamSender {
public:
	StreamSender(boost::asio::io_context& io_context);
	void set_destination_endpoint(const boost::asio::ip::udp::endpoint& endpoint);
	// Blocking send
	void send_frame(int stream, uint8_t* data, std::size_t len);
	void create_streams(int stream_count);
	void set_max_section_size(uint32_t max);
	inline uint32_t get_max_section_size() const { return max_section_size; }
private:
	std::vector<StreamMetadata> stream_info;
	boost::asio::io_context& ctx;
	boost::asio::ip::udp::socket socket;
	boost::asio::ip::udp::endpoint destination;
	uint32_t max_section_size = 1024;
};

// Forward declaration for Frame (small cross-dependency for "friend" declaration)
class StreamReceiver;

// Provides thread-safe, RAII-style access to completed frames
// Data is valid until calling release() or until going out of scope
// Object cannot be copied
class Frame {
public:
	Frame() = default;
	Frame(Frame&& src);

	// Since Frame owns the completion_lock, do not allow copying
	Frame(const Frame&) = delete;
	~Frame();

	// Unlock the completion_lock, allowing StreamReceiver to overwrite this frame buffer if needed
	void release();

	inline std::size_t size() const { return len; }
	inline uint8_t* data() { return _data; }
	inline const uint8_t* data() const { return _data; }
	inline uint8_t& operator[](std::size_t i) { return _data[i]; }
	inline uint8_t operator[](std::size_t i) const { return _data[i]; }
private:
	uint8_t* _data = nullptr;
	std::size_t len = 0;
	std::mutex* completion_lock = nullptr;

	friend StreamReceiver;
	void bind(std::mutex* completion_lock, uint8_t* data, std::size_t len);
};

// Partial thread-safety:
//	- io_context handlers may be dispatched concurrently
//	- control functions (open_stream, close_stream, etc.) may not be called concurrently
// Intended design: 1 main thread, 1 io thread, 1 frame processing thread
class StreamReceiver {
private:
	// Private: Internal organizational structures

	// Metadata and pointers for frame buffers allocated by Stream
	struct FrameBuf {
		uint8_t* data;
		std::size_t received_size;
		uint8_t received_sections;
		uint8_t frame_index;
	};
	
	// Hold buffers and other metadata necessary for reconstructing a single stream
	struct Stream {
		// Allocate and use n buffers for reconstructing this stream. Any previously buffered data is lost
		void alloc_buffers(std::size_t size, int n);
		void free_buffers();
		Stream(Stream&& src);
		Stream();
		~Stream();

		// Buffering design:
		// Use n equal-sized frame buffers to place sections when they are received
		// When all sections of a frame are received into a buffer, mark that buffer as complete
		// The complete buffer is not overwritten until another buffer becomes complete
		// 
		// Thread safety:
		// Certain operations (decoding) must be assured the complete buffer is not overwritten while in use
		// Lock completion_lock before changing complete_buffer -or- accessing FrameBuf[complete_buffer]
		// The Frame object hides this from users:
		//	- StreamReceiver::get_complete_frame locks the completion_lock and packages it in a Frame obj
		//	- Users have exclusive access to Frame while in scope
		//	- Frame's destructor releases the lock

		// All buffer sizes are equal, so allocate as one contiguous block
		std::size_t indv_buffer_size;
		uint8_t* all_data = nullptr;
		std::vector<FrameBuf> frame_buffers;

		// Lock when the complete buffer is in use to prevent overwriting when another frame completes
		std::mutex completion_lock;

		// Which buffer is complete?
		int complete_buffer = -1;
		bool open = false;
	};
public:

	// Placeholder aliases for frame receipt callback
	// @param stream_index Stream with a complete frame
	// @param frame The completed frame
	struct args {
		typedef decltype(std::placeholders::_1) stream_index;
		typedef decltype(std::placeholders::_2) frame;
	};

	StreamReceiver(boost::asio::io_context& io_context);

	// Regular Mode
	void set_listen_port(uint16_t port);
	void begin(uint16_t port);

	// Multicast Mode
	void subscribe(boost::asio::ip::udp::endpoint& feed);

	// Open stream and allocate buffers if needed
	void open_stream(int stream);
	// Close stream but leave buffers
	void close_stream(int stream);
	// Close stream and free buffers
	void destroy_stream(int stream);

	void set_frame_buffer_params(std::size_t size, unsigned level);
	// Set the size of each stream's frame buffer
	inline void set_frame_buffer_size(std::size_t size) { set_frame_buffer_params(size, _frame_buffer_level); }
	// Set the number of buffers each stream uses for reconstructing frames
	inline void set_frame_buffer_level(unsigned level) { set_frame_buffer_params(_frame_buffer_size, level); }

	inline std::size_t frame_buffer_size() const { return _frame_buffer_size; }
	inline unsigned frame_buffer_level() const { return _frame_buffer_level; }

	// Set the buffer size used for receiving incoming sections
	void set_section_buffer_size(std::size_t);
	inline std::size_t section_buffer_size() const { return recv_buffer_size; };

	// Get exclusive access to the latest completed frame.
	// throws std::out_of_range if stream is invalid
	// throws std::range_error if no frame is available
	Frame get_complete_frame(int stream);
	inline void on_frame_received(std::function<void(int stream, Frame& frame)> handler) { frame_handler = handler; }

private:
	boost::asio::io_context& ctx;
	boost::asio::ip::udp::socket socket;
	boost::asio::ip::udp::endpoint remote;
	std::unique_ptr<uint8_t> recv_buffer;
	// start with default size; allocates on write
	std::size_t recv_buffer_size = 2048;
	std::vector<Stream> streams;
	// Reader-writer lock: stream vector cannot be reallocated while being read
	std::shared_mutex streams_lock;
	std::function<void(int stream, Frame& frame)> frame_handler;
	// Default size: 4MB
	unsigned _frame_buffer_size = 4 * 1024 * 1024;
	unsigned _frame_buffer_level = 3;
	uint16_t port;
	void receive();

};

// Identifies information needed to reconstruct multiple streams from streams split into sections
// Only 3 LSB of offset
struct FrameHeader {
	static constexpr std::size_t SIZE = 4 * sizeof(int8_t) + 3;
	int8_t stream_index;
	uint8_t frame_index;
	uint8_t section_index;
	uint8_t section_count;
	// Max: 16 MB
	uint32_t offset;
	void write(uint8_t* arr) const;
	void read(const uint8_t* arr);
	void write_new_section(uint8_t* arr) const;
};

} // end namespace net