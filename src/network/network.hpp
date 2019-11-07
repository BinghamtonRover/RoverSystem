#ifndef NETWORK_H
#define NETWORK_H

#include <stddef.h>
#include <stdint.h>

#include "../util/util.hpp"

#include "memory.hpp"

namespace network {

//
// Definitions
//

const uint8_t PROTOCOL_VERSION = 1;

const int MAX_RAW_PACKET_SIZE = 1472; // Use Ethernet MTU.
// Header:
//     u8  protocol_version
//     u8  message_type
//     u16 message_size
const int HEADER_SIZE = 1 + 1 + 2;
const int MAX_MESSAGE_SIZE = MAX_RAW_PACKET_SIZE - HEADER_SIZE;

// The time a sending feed will wait with no sending before sending a heartbeat.
const int MAX_IDLE_TIME = 1000; // In ms.
// The max time a receiving feed will wait for incoming packets before declaring
// a feed to be DEAD.
const int MAX_HEARTBEAT_WAIT_TIME = 2000;

// TODO: If things aren't getting through, increase this number to 32.
const uint8_t MULTICAST_TTL = 64;

#define NETWORK_ERROR_DEF(X) \
    X(OK), X(NOMORE), \
\
        X(SOCKET), X(BIND), X(MULTICAST_JOIN), X(CONNECT), X(SEND), X(RECEIVE), X(VERSION), X(INVALID_ADDRESS), X(INVALID_INTERFACE)

#define X_IDENTITY(name) name
enum class Error { NETWORK_ERROR_DEF(X_IDENTITY) };
#undef X_IDENTITY

const char* get_error_string(Error error);

//
// End Definitions
//

//
// Feed Management
//

enum class FeedStatus { ALIVE, DEAD };

enum class FeedType { IN, OUT };

struct Feed {
    FeedType type;
    int socket_fd;

    MemoryPool memory_pool;

    util::Clock* clock;

    int bytes_transferred;
    FeedStatus status;
    uint32_t last_active_time;
};

Error init(Feed* out_feed, FeedType type, const char* interface, const char* group, uint16_t port, util::Clock* clock);

Error update_status(Feed* feed);

void close(Feed* feed);

//
// End Feed Management
//

//
// Buffer Stuff
//

struct Buffer {
    uint16_t index;
    uint16_t size;
    uint16_t cap;

    uint8_t* data;
};

template <typename T>
void deserialize(Buffer* buffer, T* t) {
    t->deserialize(buffer);
}

void serialize(Buffer* buffer, uint8_t value);
void deserialize(Buffer* buffer, uint8_t* value);

void serialize(Buffer* buffer, uint16_t value);
void deserialize(Buffer* buffer, uint16_t* value);

void serialize(Buffer* buffer, int16_t value);
void deserialize(Buffer* buffer, int16_t* value);

void serialize(Buffer* buffer, uint32_t value);
void deserialize(Buffer* buffer, uint32_t* value);

void serialize(Buffer* buffer, int32_t value);
void deserialize(Buffer* buffer, int32_t* value);

void serialize(Buffer* buffer, uint64_t value);
void deserialize(Buffer* buffer, uint64_t* value);

void serialize(Buffer* buffer, int64_t value);
void deserialize(Buffer* buffer, int64_t* value);

void serialize(Buffer* buffer, bool value);
void deserialize(Buffer* buffer, bool* value);

void serialize(Buffer* buffer, float value);
void deserialize(Buffer* buffer, float* value);

void serialize(Buffer* buffer, double value);
void deserialize(Buffer* buffer, double* value);

void serialize(Buffer* buffer, uint8_t* memory, size_t size);
void deserialize(Buffer* buffer, uint8_t* memory, size_t out_size);

Buffer get_outgoing_buffer(Feed* feed);

//
// End Buffer Stuff
//

//
// Message Definitions
//

enum class MessageType { 
    HEARTBEAT, 
    TEST, 
    MOVEMENT, 
    CAMERA, 
    CAMERA_CONTROL,
    LOG, 
    LIDAR, 
    LOCATION, 
    TICK,
    SUBSYSTEM
};

struct HeartbeatMessage {
    static const auto TYPE = MessageType::HEARTBEAT;

    void serialize(Buffer* buffer) {
    }
    void deserialize(Buffer* buffer) {
    }
};

struct TestMessage {
    static const auto TYPE = MessageType::TEST;

    uint16_t message_size;
    uint8_t message[MAX_MESSAGE_SIZE];

    void serialize(Buffer* buffer) {
        network::serialize(buffer, this->message_size);
        network::serialize(buffer, this->message, this->message_size);
    }

    void deserialize(Buffer* buffer) {
        network::deserialize(buffer, &(this->message_size));
        network::deserialize(buffer, this->message, this->message_size);
    }
};

struct MovementMessage {
    static const auto TYPE = MessageType::MOVEMENT;

    int16_t left, right;

    void serialize(Buffer* buffer) {
        network::serialize(buffer, this->left);
        network::serialize(buffer, this->right);
    }

    void deserialize(Buffer* buffer) {
        network::deserialize(buffer, &(this->left));
        network::deserialize(buffer, &(this->right));
    }
};

struct CameraMessage {
    static const auto TYPE = MessageType::CAMERA;

    static const int HEADER_SIZE = 7;

    uint8_t stream_index;
    uint16_t frame_index;
    uint8_t section_index;
    uint8_t section_count;
    uint16_t size;

    uint8_t* data;

    void serialize(Buffer* buffer) {
        network::serialize(buffer, this->stream_index);
        network::serialize(buffer, this->frame_index);
        network::serialize(buffer, this->section_index);
        network::serialize(buffer, this->section_count);
        network::serialize(buffer, this->size);
        network::serialize(buffer, this->data, this->size);
    }

    void deserialize(Buffer* buffer) {
        network::deserialize(buffer, &(this->stream_index));
        network::deserialize(buffer, &(this->frame_index));
        network::deserialize(buffer, &(this->section_index));
        network::deserialize(buffer, &(this->section_count));
        network::deserialize(buffer, &(this->size));
        network::deserialize(buffer, this->data, this->size);
    }
};

struct LogMessage {
    static const auto TYPE = MessageType::LOG;

    uint8_t size;
    uint8_t* log_string;

    void serialize(Buffer* buffer) {
        network::serialize(buffer, this->size);
        network::serialize(buffer, this->log_string, this->size);
    }

    void deserialize(Buffer* buffer) {
        network::deserialize(buffer, &(this->size));
        network::deserialize(buffer, this->log_string, this->size);
    }
};

const int NUM_LIDAR_POINTS = 271;

struct LidarMessage {
    static const auto TYPE = MessageType::LIDAR;

    uint16_t points[NUM_LIDAR_POINTS];

    void serialize(Buffer* buffer) {
        for (int i = 0; i < NUM_LIDAR_POINTS; i++) {
            network::serialize(buffer, points[i]);
        }
    }

    void deserialize(Buffer* buffer) {
        for (int i = 0; i < NUM_LIDAR_POINTS; i++) {
            network::deserialize(buffer, this->points + i);
        }
    }
};

struct LocationMessage {
    static const auto TYPE = MessageType::LOCATION;

    bool has_fix;
    float latitude;
    float longitude;

    void serialize(Buffer* buffer) {
        network::serialize(buffer, this->has_fix);
        network::serialize(buffer, this->latitude);
        network::serialize(buffer, this->longitude);
    }

    void deserialize(Buffer* buffer) {
        network::deserialize(buffer, &(this->has_fix));
        network::deserialize(buffer, &(this->latitude));
        network::deserialize(buffer, &(this->longitude));
    }
};

struct CameraControlMessage {
    static const auto TYPE = MessageType::CAMERA_CONTROL;
    enum Setting : uint8_t {
        JPEG_QUALITY,
        GREYSCALE, /* E for consistancy */
        DISPLAY_STATE
    };

    enum sendType : uint8_t {
        DONT_SEND,
        SEND_SMALL,
        SEND_LARGE
    };

    Setting setting;

    union {
        uint8_t jpegQuality;
        bool greyscale;
        struct {
            uint8_t stream_index;
            sendType sending;
        } resolution;
    };

    void serialize(Buffer* buffer) {
        network::serialize(buffer, static_cast<uint8_t>(this->setting));
        switch(this->setting){
            case JPEG_QUALITY:
                network::serialize(buffer, this->jpegQuality);
                break;
            case GREYSCALE:
                network::serialize(buffer, this->greyscale);
                break;
            case DISPLAY_STATE:
                network::serialize(buffer, this->resolution.stream_index);
                network::serialize(buffer, static_cast<uint8_t>(
                                            this->resolution.sending));
                break;
        }
        
    }
    void deserialize(Buffer* buffer) {
        network::deserialize(buffer, reinterpret_cast<uint8_t *>(&(this->setting)));
        switch(this->setting){
            case JPEG_QUALITY:
                network::deserialize(buffer, &(this->jpegQuality));
                break;
            case GREYSCALE:
                network::deserialize(buffer, &(this->greyscale));
                break;
            case DISPLAY_STATE:
                network::deserialize(buffer, 
                                    &(this->resolution.stream_index));
                network::deserialize(buffer, reinterpret_cast<uint8_t *>(&(this->resolution.sending)));
                break;
        }
    }
};

struct TickMessage {
    static const auto TYPE = MessageType::TICK;

    float ticks_per_second;

    void serialize(Buffer* buffer) {
        network::serialize(buffer, this->ticks_per_second);
    }

    void deserialize(Buffer* buffer) {
        network::deserialize(buffer, &(this->ticks_per_second));
    }
};

struct SubsystemMessage {
    static const auto TYPE = MessageType::SUBSYSTEM;
    static const auto MAX_NAME_LEN = 32;

    struct Subsystem {
        uint8_t id;
       
        // Null-terminated.
        uint8_t name[MAX_NAME_LEN + 1];

        uint8_t state; 

        // Takes a null-terminated string and truncates it as necessary.
        void set_name(const char* name);
    };

    uint8_t num;
    Subsystem* subsystems;

    void serialize(Buffer* buffer) {
        network::serialize(buffer, this->num);

        for (uint8_t i = 0; i < this->num; i++) {
            network::serialize(buffer, this->subsystems[i].id);
            network::serialize(buffer, this->subsystems[i].name, MAX_NAME_LEN + 1);
            network::serialize(buffer, this->subsystems[i].state);
        }
    }

    void deserialize(Buffer* buffer) {
        network::deserialize(buffer, &(this->num));

        for (uint8_t i = 0; i < this->num; i++) {
            network::deserialize(buffer, &(this->subsystems[i].id));
            network::deserialize(buffer, this->subsystems[i].name, MAX_NAME_LEN + 1);
            network::deserialize(buffer, &(this->subsystems[i].state));
        }
    }
};

//
// End Message Definitions
//

//
// Sending and Receiving
//

struct IncomingMessage {
    MessageType type;
    Buffer buffer;
};

Error receive(Feed* feed, IncomingMessage* out_message);
Error send(Feed* feed, MessageType type, Buffer buffer);

template <typename T>
Error publish(Feed* feed, T* thing) {
    auto buffer = get_outgoing_buffer(feed);
    auto message_type = T::TYPE;
    thing->serialize(&buffer);

    return send(feed, message_type, buffer);
}

//
// End Sending and Receiving
//

} // namespace network

#endif
