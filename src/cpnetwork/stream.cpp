#include "stream.hpp"

#include <cstring>
#include <limits>
#include <iostream>
#include <boost/array.hpp>

net::StreamSender::StreamSender(boost::asio::io_context& io_context) : ctx(io_context), socket(ctx) {
	stream_info.reserve(8);
	socket.open(boost::asio::ip::udp::v4());
}

void net::StreamSender::set_destination_endpoint(const boost::asio::ip::udp::endpoint& endpoint) {
	destination = endpoint;
}

void net::StreamSender::create_streams(int stream_count) {
	if (stream_info.size() < stream_count) {
		stream_info.resize(stream_count);
	}
}

void net::StreamSender::send_frame(int stream, uint8_t* data, std::size_t len) {
	if (stream >= stream_info.size() || stream < 0) {
		throw std::out_of_range("net::StreamSender::send_frame: stream index out of range");
	}
	FrameHeader hdr;
	uint8_t header_buffer[FrameHeader::SIZE];
	auto io_header_buffer = boost::asio::buffer(header_buffer, FrameHeader::SIZE);

	// aways need 1 more section than (len / max_section_size) unless they divide perfectly
	hdr.section_count = len / max_section_size + !!(len % max_section_size);
	hdr.stream_index = stream;
	hdr.frame_index = stream_info[stream].frame_index++;
	hdr.section_index = 0;
	hdr.offset = 0;

	hdr.write(header_buffer);

	while (len > 0) {
		
		std::size_t n_sent = (len > max_section_size) ? max_section_size : len;
		hdr.write_new_section(header_buffer);

		auto io_section_buffer = boost::asio::buffer(data, n_sent);
		boost::array<decltype(io_header_buffer), 2> io_buffers = {io_header_buffer, io_section_buffer};

		try {
			std::size_t act_sent = socket.send_to(io_buffers, destination) - FrameHeader::SIZE;
			if (act_sent != n_sent) break;
			len -= n_sent;
			data = &data[n_sent];

			hdr.section_index++;
			hdr.offset += n_sent;
			hdr.write_new_section(header_buffer);
		} catch (const boost::system::system_error& error) {
			// Cancel sending the frame on error
			break;
		}
	}
}

void net::StreamSender::set_max_section_size(uint32_t max) {
	if (max < 0x00FFFFFF && max > 0) max_section_size = max;
}

void net::FrameHeader::write(uint8_t* arr) const {
	arr[0] = stream_index;
	arr[1] = frame_index;
	arr[2] = section_index;
	arr[3] = section_count;

	arr[4] = offset & 0xFF;
	arr[5] = (offset >> 8) & 0xFF;
	arr[6] = (offset >> 16) & 0xFF;
}

void net::FrameHeader::read(const uint8_t* arr) {
	stream_index = arr[0];
	frame_index = arr[1];
	section_index = arr[2];
	section_count = arr[3];

	offset = arr[4] | (arr[5] << 8) | (arr[6] << 16);
}

void net::FrameHeader::write_new_section(uint8_t* arr) const {
	arr[2] = section_index;

	arr[4] = offset & 0xFF;
	arr[5] = (offset >> 8) & 0xFF;
	arr[6] = (offset >> 16) & 0xFF;
}

net::StreamReceiver::Stream::Stream() { }

// When moving object, no need to reallocate the buffers
// Basically the default move constructor but src.all_data set to null so it doesn't get destructed
// Also note that completion_lock cannot be copied:
//	Reading stream data and moving stream objects must be mutually exclusive.
//	Enforce with higher-level locks (eg. StreamReceiver::streams_lock)
net::StreamReceiver::Stream::Stream(Stream&& src) :
	indv_buffer_size(std::move(src.indv_buffer_size)),
	all_data(std::move(src.all_data)),
	frame_buffers(std::move(src.frame_buffers)),
	complete_buffer(std::move(src.complete_buffer)),
	open(std::move(src.open)) {

	src.all_data = nullptr;
}

net::StreamReceiver::Stream::~Stream() {
	delete[] all_data;
}

void net::StreamReceiver::Stream::alloc_buffers(std::size_t buf_size, int buf_level) {
	// Do not reallocate if they are the same size
	if (all_data && buf_level == frame_buffers.size() && buf_size == indv_buffer_size) return;

	std::lock_guard lock(completion_lock);

	if (all_data) {
		delete[] all_data;
	}

	all_data = new uint8_t[buf_size * buf_level];
	indv_buffer_size = buf_size;

	frame_buffers.resize(buf_level);
	std::size_t offset = 0;
	for (auto& f : frame_buffers) {
		f.data = &all_data[offset];
		f.received_size = 0;
		f.received_sections = 0;

		offset += buf_size;
	}
	complete_buffer = -1;
}

void net::StreamReceiver::Stream::free_buffers() {
	std::lock_guard locked(completion_lock);

	delete[] all_data;
	all_data = nullptr;
	complete_buffer = -1;

	frame_buffers.clear();

}

net::StreamReceiver::StreamReceiver(boost::asio::io_context& io_context) : ctx(io_context), socket(io_context) {
	streams.reserve(8);
}

void net::StreamReceiver::set_listen_port(uint16_t port) {
	this->port = port;
	if (socket.is_open()) {
		socket.close();
		begin(port);
	}
}

void net::StreamReceiver::begin(uint16_t port) {
	socket.open(boost::asio::ip::udp::v4());
	socket.bind(boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), port));

	receive();
}

void net::StreamReceiver::subscribe(boost::asio::ip::udp::endpoint& ep) {
	if (socket.is_open()) {
		socket.close();
	}

	socket.open(ep.protocol());
	socket.set_option(boost::asio::ip::udp::socket::reuse_address(true));
	socket.bind(boost::asio::ip::udp::endpoint(
		boost::asio::ip::address_v4::from_string("0.0.0.0"),
		ep.port()
	));

	socket.set_option(boost::asio::ip::multicast::join_group(ep.address()));

	receive();
}

void net::StreamReceiver::receive() {
	if (!recv_buffer.get()) {
		recv_buffer.reset(new uint8_t[recv_buffer_size]);
	}
	socket.async_receive_from(boost::asio::buffer(recv_buffer.get(), recv_buffer_size), remote, [this](auto error, auto bytes_transferred) {
		if (!error && bytes_transferred >= FrameHeader::SIZE) {
			FrameHeader section;
			section.read(recv_buffer.get());

			// Acquire streams_lock as a reader
			std::shared_lock<std::shared_mutex> streams_reader(streams_lock);
			if (section.stream_index >= 0 && section.stream_index < streams.size() && streams[section.stream_index].open) {
				
				Stream& s = streams[section.stream_index];

				// One buffer is reserved as a "completed frame" buffer, pick from the others
				int use_buffer = section.frame_index % (s.frame_buffers.size() - 1);
				// Do not overwrite most recently completed frame
				if (s.complete_buffer != -1 && use_buffer >= s.complete_buffer) {
					use_buffer++;
					if (use_buffer == s.frame_buffers.size()) use_buffer = 0;
				}

				// Continue reconstructing this frame -or- overwrite the old frame
				FrameBuf& f = s.frame_buffers[use_buffer];
				if (f.frame_index != section.frame_index) {
					// Different frames; overwrite
					f.frame_index = section.frame_index;
					f.received_sections = 0;
					f.received_size = 0;
				}

				std::size_t data_bytes_in = bytes_transferred - FrameHeader::SIZE;
				if (section.offset + data_bytes_in <= s.indv_buffer_size) {
					std::memcpy(&f.data[section.offset], &recv_buffer.get()[FrameHeader::SIZE], data_bytes_in);
					f.received_sections++;
					f.received_size += data_bytes_in;
					if (f.received_sections == section.section_count) {
						// Frame is now complete
						// Cannot change completion pointer if the old complete buffer is in use (lock)
						s.completion_lock.lock();
						s.complete_buffer = use_buffer;

						Frame completed;
						// Transfer ownership to Frame
						completed.bind(&s.completion_lock, f.data, f.received_size);

						if (frame_handler) frame_handler(section.stream_index, completed);

					}
				}

			}
		}
		receive();
	});
}

void net::StreamReceiver::open_stream(int stream) {
	// Direct map stream index to an entry in streams. Ensure table is big enough
	if (stream >= streams.size()) {
		std::unique_lock<std::shared_mutex> writer(streams_lock);
		streams.resize(stream + 1);
	}
	streams[stream].alloc_buffers(_frame_buffer_size, _frame_buffer_level);
	streams[stream].open = true;
}

void net::StreamReceiver::destroy_stream(int stream) {
	if (stream < streams.size()) {
		std::unique_lock streams_writer(streams_lock);
		streams[stream].free_buffers();
		streams[stream].open = false;
	}
}

void net::StreamReceiver::close_stream(int stream) {
	if (stream < streams.size()) {
		streams[stream].open = false;
	}
}

void net::StreamReceiver::set_frame_buffer_params(std::size_t size, unsigned level) {
	if (size > static_cast<uint64_t>(std::numeric_limits<unsigned>::max()))
		throw std::out_of_range("net::StreamReceiver::set_frame_buffer_params: max size is 4GB");

	if (level < 2)
		throw std::invalid_argument("net::StreamReceiver::set_frame_buffer_params: minimum buffer level is 2");

	std::unique_lock streams_writer(streams_lock);

	_frame_buffer_size = size;
	_frame_buffer_level = level;

	for (auto& stream : streams) {
		stream.alloc_buffers(_frame_buffer_size, _frame_buffer_level);
	}
}

void net::StreamReceiver::set_section_buffer_size(std::size_t size) {
	if (size == recv_buffer_size) return;

	bool reopen = false;
	if (socket.is_open()) {
		socket.close();
		reopen = true;
	}

	recv_buffer_size = size;
	recv_buffer.reset(new uint8_t[recv_buffer_size]);

	if (reopen)
		begin(port);

}

net::Frame net::StreamReceiver::get_complete_frame(int stream) {
	std::shared_lock<std::shared_mutex> streams_reader(streams_lock);
	if (stream >= streams.size())
		throw std::out_of_range("net::StreamReceiver::get_complete_frame: stream index out of range");

	Stream& s = streams[stream];
	s.completion_lock.lock();

	if (s.complete_buffer == -1)
		throw std::range_error("net::StreamReceiver::get_complete_frame: no frame available");

	// completion_lock is intentionally left unlocked: Frame's destructor unlocks automatically
	// This guarantees get_complete_frame callers have exclusive access to Frame until it is destroyed
	Frame completed_frame;
	completed_frame.bind(&s.completion_lock, &s.all_data[s.complete_buffer * s.indv_buffer_size], s.frame_buffers[s.complete_buffer].received_size);
	return completed_frame;

}

net::Frame::Frame(Frame&& src) :
	_data(std::move(src._data)),
	len(std::move(src.len)),
	completion_lock(std::move(src.completion_lock)) {

	src.completion_lock = nullptr;

}

net::Frame::~Frame() {
	if (completion_lock) {
		completion_lock->unlock();
	}
}

void net::Frame::bind(std::mutex* completion_lock, uint8_t* data, std::size_t len) {
	this->_data = data;
	this->completion_lock = completion_lock;
	this->len = len;
}

void net::Frame::release() {
	completion_lock->unlock();
	completion_lock = nullptr;
}
