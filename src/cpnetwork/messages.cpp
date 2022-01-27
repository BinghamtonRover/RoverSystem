#include "messages.hpp"

unsigned int msg::MESSAGE_TYPE_COUNT = 1;

msg::Header::Header(const uint8_t* arr) {
	read(arr);
}

msg::Header::Header(type_t type, int size) : type(type), size(size) { }

void msg::Header::write(uint8_t* arr) const {
	arr[0] = size;
	arr[1] = size >> 8;
	arr[2] = static_cast<uint8_t>(type);
}

void msg::Header::read(const uint8_t* arr) {
	size = arr[0];
	size |= arr[1] << 8;
	type = static_cast<type_t>(arr[2]);
	if (type > MESSAGE_TYPE_COUNT) {
		type = TYPE_NONE;
	}
}


void msg::Receiver::create_handlers() {
	receipt_handlers.resize(MESSAGE_TYPE_COUNT - 1);
}

void msg::Receiver::read_messages(const uint8_t* buf, std::size_t size) {
	for (std::size_t i = 0; i + Header::HDR_SIZE <= size; /**/) {
		Header hdr(&buf[i]);
		i += Header::HDR_SIZE;

		if (hdr.type != TYPE_NONE && i + hdr.size <= size) {
			Handler& h = receipt_handlers[hdr.type - 1];
			if (h) h(&buf[i], hdr.size);
			i += hdr.size;
		} else {
			break;
		}
	}
}

void msg::Receiver::register_handler(type_t type, Handler handler) {
	if (receipt_handlers.size() != MESSAGE_TYPE_COUNT - 1) {
		create_handlers();
	}

	if (type < MESSAGE_TYPE_COUNT && type != TYPE_NONE) {
		receipt_handlers[type - 1] = handler;
	}
}
