#include <assert.h>
#include <stdlib.h>
#include <string.h>

#include "memory.hpp"

namespace network {

struct ElementHeader {
    uint8_t bucket;
    uint8_t index_in_bucket;
};

void MemoryPool::init(int element_size, int elements_per_bucket) {
    this->element_size = element_size;
    this->segment_size = element_size + sizeof(ElementHeader);
    this->elements_per_bucket = elements_per_bucket;

    // Start with one bucket.
    this->num_buckets = 1;

    // 0 indicates full (or unused).
    memset(this->bucket_bitmaps, 0, sizeof(this->bucket_bitmaps));
    this->bucket_bitmaps[0] = ~(0x0);

    memset(this->buckets, 0, sizeof(this->buckets));
    this->buckets[0] = (uint8_t*) malloc(this->segment_size * this->elements_per_bucket);
}

void MemoryPool::close() {
    for (int i = 0; i < this->num_buckets; i++) {
        free(this->buckets[i]);
    }
}

static int rightmost_bit(uint64_t n) {
    return (int) (n & ~(n - 1)) - 1;
}

uint8_t* MemoryPool::alloc() {
    uint8_t* segment_ptr = nullptr;
    ElementHeader header;

    for (int i = 0; i < this->num_buckets; i++) {
        auto rmb = rightmost_bit(this->bucket_bitmaps[i]);
        if (rmb == -1) continue;

        header = { (uint8_t) i, (uint8_t) rmb };
        segment_ptr = this->buckets[i] + (this->segment_size) * rmb;
        break;
    }

    if (segment_ptr == nullptr) {
        // Allocate more?
        assert(this->num_buckets < 64);

        this->bucket_bitmaps[this->num_buckets] = ~(0x0);
        this->buckets[this->num_buckets] = (uint8_t*) malloc(this->segment_size * this->elements_per_bucket);

        header = { (uint8_t) this->num_buckets, 0 };
        segment_ptr = this->buckets[this->num_buckets];

        this->num_buckets += 1;
    }

    this->bucket_bitmaps[header.bucket] &= ~(1 << header.index_in_bucket);

    *reinterpret_cast<ElementHeader*>(segment_ptr) = header;

    return segment_ptr + sizeof(ElementHeader);
}

void MemoryPool::free(uint8_t* element) {
    uint8_t* segment_ptr = element - sizeof(ElementHeader);

    auto header = *reinterpret_cast<ElementHeader*>(segment_ptr);

    this->bucket_bitmaps[header.bucket] |= 1 << header.index_in_bucket;
}

} // namespace network
