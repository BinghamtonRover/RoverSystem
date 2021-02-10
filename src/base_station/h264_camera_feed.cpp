#include "h264_camera_feed.hpp"

#include <cstring>

namespace camera_feed {

H264Frame::~H264Frame() {
    reset();
}

void H264Frame::insert_partial_frame(uint8_t* partial_frame, uint16_t size, uint8_t section_index) {
    ++num_received;
    if (size > 0) {
        partial_frames[section_index] = partial_frame_t(partial_frame, size);
    } else {
        // The end-of-frame is marked by a zero-size message
        expected_size = section_index + 1;
    }
}

bool H264Frame::is_complete() const {
    return num_received != 0 && num_received == expected_size;
}

uint8_t* H264Frame::make_frame(size_t& size) {
    size = 0;
    for (uint i = 0; i < num_received; ++i) {
        size += partial_frames[i].second;
    }
    uint8_t* frame = new uint8_t[size];
    uint8_t* current = frame;
    for (int i = 0; i < num_received; ++i) {
        memcpy(current, partial_frames[i].first, partial_frames[i].second);
        current += partial_frames[i].second;
    }
    return frame;
}

void H264Frame::reset() {
    for (uint i = 0; i < partial_frames.size(); ++i) {
        if (partial_frames[i].first != nullptr)
            delete[] partial_frames[i].first;
    }
    num_received = 0;
    expected_size = 0;
}

bool H264Frame::is_older_than(uint16_t other_frame_index) {
    return num_received == 0 ||
        ((frame_index > other_frame_index) && (frame_index - other_frame_index <= 32768)) ||
        ((frame_index < other_frame_index) && (other_frame_index - frame_index > 32768));
}

void H264Feed::handle_message(network::CSICameraMessage& message) {
    size_t use_buffer = message.frame_index % num_buffers;

    if (frame_buffers[use_buffer].is_older_than(message.frame_index)) {
        frame_buffers[use_buffer].reset();
        frame_buffers[use_buffer].insert_partial_frame(message.data, message.size, message.section_index);
        frame_buffers[use_buffer].set_frame_index(message.frame_index);

        if (frame_buffers[use_buffer].is_complete()) {
            size_t frame_size;
            uint8_t* frame = frame_buffers[use_buffer].make_frame(frame_size);
            // TODO decode this frame
            delete[] frame;
        }
    }
}

} // end namespace camera_feed
