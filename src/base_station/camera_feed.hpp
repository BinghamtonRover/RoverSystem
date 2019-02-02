#include <cstdint>

namespace camera_feed {

const int CAMERA_FRAME_BUFFER_SIZE = 6220800;
const int CAMERA_FRAME_BUFFER_COUNT = 5;

struct BufferItem {
    uint16_t frame_index;
    uint16_t data_size;
    uint8_t sections_remaining;
    unsigned char* data_sections;
};

struct Feed {
    int width, height;

    unsigned int gl_texture_id;

    BufferItem* buffers;
};

enum class Error {
    OK,

    TURBO_JPEG_INIT,
    DECOMPRESS
};

Error init();

Error init_feed(Feed* feed, int width, int height);

Error handle_section(Feed* feed, unsigned char* data, int data_size, int section_index, int section_count, int frame_index);

void destroy_feed(Feed* feed);
}