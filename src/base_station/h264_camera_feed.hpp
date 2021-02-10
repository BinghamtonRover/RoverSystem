#ifndef H264_CAMERA_FEED_HPP
#define H264_CAMERA_FEED_HPP

#include "../network/network.hpp"

#include <array>
#include <utility>

namespace camera_feed {

// Typedef for brevity
// First is the data, second is its size
typedef std::pair<uint8_t*, uint16_t> partial_frame_t;

class H264Frame {
public:
    ~H264Frame();

    // Add a new partial frame that is part of this
    // Consider H264Frame as "taking ownership" of the partial_frame buffer after calling this
    void insert_partial_frame(uint8_t* partial_frame, uint16_t size, uint8_t section_index);

    // Have all the expected sections been receieved?
    bool is_complete() const;

    // Turn all partial frames into one buffer. Calling on incomplete frame is undefined
    // Returns pointer to a *dynamically allocated* buffer, 
    uint8_t* make_frame(size_t& size);

    // Allows for reuse of the same H264Frame instance
    void reset();

    // Returns true if this frame is older than other frame OR if this frame is unused
    bool is_older_than(uint16_t other_frame_index);

    inline void set_frame_index(uint16_t index) {
        frame_index = index;
    }
    inline uint16_t get_frame_index() const {
        return frame_index;
    }

private:
    // A frame is sent as buffers of smaller, partial frames
    // There are a maximum of 256 partial frames per frame
    std::array<partial_frame_t, 256> partial_frames{};

    uint8_t expected_size = 0;
    uint8_t num_received = 0;
    uint16_t frame_index = 0;
};

class H264Feed {
public:
    static constexpr size_t num_buffers = 10;
    void handle_message(network::CSICameraMessage& message);
    uint32_t width, height;

private:
    std::array<H264Frame, num_buffers> frame_buffers{};
};

} // end namespace camera_feed

#endif
