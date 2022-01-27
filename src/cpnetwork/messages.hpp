/*
	Message serialization/deserialization tools

	This can be used with any transport medium, not only the network library.

	Message types must be generated with Google Protocol Buffers. Library users
	can use macro DEFINE_MESSAGE_TYPE(NewClassName, ProtobufMessageTypeName) to
	create define a struct NewClassName for each message type. Then, register
	messages with msg::register_message_type<NewClassName>() to generate a
	unique type enumerator at runtime. Type registration is global but may be
	changed in the future.
*/

#pragma once

#define DEFINE_MESSAGE_TYPE(T, MSG_T) struct T : public msg::Message { MSG_T data; inline static msg::type_t TYPE; T() { data_p = &data; type = T::TYPE; }};

#include <cstdint>
#include <limits>
#include <functional>
#include <vector>
#include <google/protobuf/message.h>

namespace msg {

typedef uint8_t type_t;
typedef uint16_t size_t;

// Number of message types registered with the network library
// This is not constant to allow dynamic calculation of the type enumerator
// It should not be changed externally
extern unsigned int MESSAGE_TYPE_COUNT;
inline unsigned int count_message_types() { return MESSAGE_TYPE_COUNT; }
constexpr type_t TYPE_NONE = 0;

template<typename T>
inline void register_message_type() {
	T::TYPE = MESSAGE_TYPE_COUNT++;
}

struct Message {
	google::protobuf::Message* data_p;
	type_t type;
};

struct Header {
	constexpr static std::size_t HDR_SIZE = sizeof(type_t) + sizeof(size_t);
	constexpr static size_t MAX_MSG_SIZE = std::numeric_limits<size_t>::max();

	type_t type;
	size_t size;

	void write(uint8_t* arr) const;
	void read(const uint8_t* arr);
	Header(const uint8_t* arr);
	Header(type_t type, int size);
};

class Receiver {
	public:
		typedef std::function<void(const uint8_t*, std::size_t)> Handler;
		void create_handlers();
		void read_messages(const uint8_t*, std::size_t);
		void register_handler(type_t type, Handler handler);

		template<typename T>
		inline void register_handler(Handler handler) {
			register_handler(T::TYPE, handler);
		}
	private:
		std::vector<Handler> receipt_handlers;
};

}	// end namespace msg
