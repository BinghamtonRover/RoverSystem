#include <stdint.h>

namespace network {

struct MemoryPool {
    int element_size;
    int segment_size;
    int elements_per_bucket;

    int num_buckets;

    // Bitmaps are filled least to most significant.
    uint64_t bucket_bitmaps[64];

    uint8_t* buckets[64];

    void init(int element_size, int elements_per_bucket);
    void close();

    uint8_t* alloc();
    void free(uint8_t* element);
};

} // namespace network
