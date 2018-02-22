#include <stdint.h>
#include <string.h>

#define BUFFERVALUE(buff, byte_offset, data_type) *((data_type*) &buff[byte_offset])

// Represents a simple wrapper around a byte buffer.
// This does no overflow checking.
// Its purpose is to make code cleaner and reduce the amount of nasty casts.
struct Buffer {
    uint8_t* data;

    size_t index;

    Buffer(uint8_t* _data):
        data(_data), index(0) {}

    template <class T>
    void read(T*);

    template <class T>
    T read_value();

    // Returns a pointer to the data at the current index.
    uint8_t* get_pointer() {
        return &data[index];
    }

    // Returns a pointer to the data at the current index.
    // Also increments index by the given size.    
    uint8_t* get_pointer(size_t size) {
        uint8_t* p = get_pointer();
        index += size;
        return p;
    }

    void read_bytes(uint8_t* d, size_t size) {
        memcpy(d, &data[index], size);
        index += size;
    }

    template <class T>
    void write(T*);

    template <class T>
    void write_value(T);

    void write_bytes(uint8_t* d, size_t size) {
        memcpy(&data[index], d, size);
        index += size;
    }

    void reset() {
        index = 0;
    }
};

template <class T>
void Buffer::read(T* t) {
    *t = BUFFERVALUE(data, index, T);
    index += sizeof(T);
}

template <class T>
T Buffer::read_value() {
    T t = BUFFERVALUE(data, index, T);
    index += sizeof(T);
    return t;
}

template <class T>
void Buffer::write(T* t) {
    BUFFERVALUE(data, index, T) = *t;
    index += sizeof(T);
}

template <class T>
void Buffer::write_value(T t) {
    BUFFERVALUE(data, index, T) = t;
    index += sizeof(T);
}