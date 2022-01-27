#pragma once

#include <vector>
#include <cstdint>

namespace net {

// Vector wrapper that prevents unncessecary memory initialization when reusing a buffer
// Provides create_block method for convenience
template<typename T>
class Buffer : public std::vector<T> {
private:
	std::size_t _usage = 0;
public:
	inline std::size_t usage() const { return _usage; }
	inline void clear() { _usage = 0; }

	void ensure_fit(std::size_t element_size) {
		if (this->size() - _usage < element_size) {
			this->resize(_usage + element_size);
		}
	}

	// Allocate a block of memory in this buffer and return a pointer to it
	T* create_block(std::size_t block_size) {
		ensure_fit(block_size);
		T* block_ptr = &this->data()[_usage];
		_usage += block_size;
		return block_ptr;
	}
};

// Maintains two buffers so that enqueue and IO send operations can happen concurrently. Class does NOT provide thread safety
template<typename T>
class DoubleBuffer {
private:
	Buffer<T> buf[2];
	uint8_t active = 0;
public:
	void swap() {
		active = !active;
	}

	inline Buffer<T>& write_buffer() {
		return buf[active];
	}
	inline Buffer<T>& read_only_buffer() {
		return buf[!active];
	}

};

}
