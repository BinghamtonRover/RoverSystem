#include "camera_feed.hpp"

#include <cstdio>
#include <cstring>

#include <GL/gl.h>

#include <turbojpeg.h>

namespace camera_feed {

tjhandle jpeg_decompressor;

// TODO: Make this dynamically-sized.
const int JPEG_SIZE = 1920 * 1080 * 3;
unsigned char decompress_buffer[JPEG_SIZE];

static Error decompress(Feed* feed, unsigned char* jpeg_data) {
    /*
       The parameters are the decompressor, the compressed image, the size of the
       image as an unsigned long, the buffer, the width of the jpeg, the pitch,
       the height, the pixel type (We have RGB here each contained in 1 byte),
       and the flags. Basically we say decompress it quickly.
   */
    if (tjDecompress2(
            jpeg_decompressor,
            jpeg_data,
            JPEG_SIZE,
            decompress_buffer,
            feed->width,
            feed->width * 3,
            feed->height,
            TJPF_RGB,
            TJFLAG_FASTDCT) == -1) {
        return Error::DECOMPRESS;
    }

    return Error::OK;
}

Error init() {
    jpeg_decompressor = tjInitDecompress();
    if (!jpeg_decompressor) {
        return Error::TURBO_JPEG_INIT;
    }

    return Error::OK;
}

Error init_feed(Feed* feed, const char* name, int width, int height) {
    strncpy(feed->name, name, FEED_NAME_MAX_LEN + 1);
    feed->width = width;
    feed->height = height;
    feed->buffers = new BufferItem[CAMERA_FRAME_BUFFER_COUNT];

    /*
       Allocates space for frame data within each frame buffer.
    */
    for (int i = 0; i < CAMERA_FRAME_BUFFER_COUNT; i++) {
        feed->buffers[i].data_sections = new unsigned char[CAMERA_FRAME_BUFFER_SIZE];
    }

    glGenTextures(1, &(feed->gl_texture_id));

    glBindTexture(GL_TEXTURE_2D, feed->gl_texture_id);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    // Initialize with an indicator of no feed.
    unsigned char* no_feed_pixels = new unsigned char[width * height * 3];
    for (int i = 0; i < width * height; i++) {
        no_feed_pixels[i * 3 + 0] = 0;
        no_feed_pixels[i * 3 + 1] = 55;
        no_feed_pixels[i * 3 + 2] = 255;
    }

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, no_feed_pixels);

    delete[] no_feed_pixels;

    return Error::OK;
}

Error handle_section(
    Feed* feed, unsigned char* data, int data_size, int section_index, int section_count, int frame_index) {
    /*
        Since we know that camera packets will only be sent out once we
        can manage the buffer by only keeping track of the number of
        sections needed. We get the index by taking the frame index mod
        the total number of buffers.
    */
    int current_frame_indx = frame_index % CAMERA_FRAME_BUFFER_COUNT;

    /*
        If we have done a full loop and there is a new
        frame index, then we overwrite the old data.
    */
    if (feed->buffers[current_frame_indx].frame_index != frame_index) {
        feed->buffers[current_frame_indx].frame_index = frame_index;
        feed->buffers[current_frame_indx].sections_remaining = section_count;
        feed->buffers[current_frame_indx].data_size = 0;
    }
    feed->buffers[current_frame_indx].data_size += data_size;
    feed->buffers[current_frame_indx].sections_remaining--;

    /*
        This section handles any data that comes out of order.
        Each data section is written at the size of
        a signle data packet times the section index.
    */
    uint32_t offset = (CAMERA_MESSAGE_FRAME_DATA_MAX_SIZE * section_index);
    memcpy(feed->buffers[current_frame_indx].data_sections + offset, data, data_size);

    if (feed->buffers[current_frame_indx].sections_remaining == 0) {
        // This frame is ready. Decompress and update the texture.

        Error err = decompress(feed, feed->buffers[current_frame_indx].data_sections);
        if (err != Error::OK) {
            return err;
        }

        glBindTexture(GL_TEXTURE_2D, feed->gl_texture_id);
        glTexImage2D(
            GL_TEXTURE_2D,
            0,
            GL_RGB,
            feed->width,
            feed->height,
            0,
            GL_RGB,
            GL_UNSIGNED_BYTE,
            decompress_buffer);
    }

    return Error::OK;
}

void destroy_feed(Feed* feed) {
    for (int i = 0; i < CAMERA_FRAME_BUFFER_COUNT; i++) {
        delete[] feed->buffers[i].data_sections;
    }

    delete[] feed->buffers;

    glDeleteTextures(1, &(feed->gl_texture_id));
}

} // namespace camera_feed
