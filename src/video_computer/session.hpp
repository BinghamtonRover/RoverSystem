#ifndef SESSION
#define SESSION

#include "../network/network.hpp"
#include "../logger/logger.hpp"
#include "../simple_config/simpleconfig.h"
#include "../util/util.hpp"

#include "camera.hpp"
#include "accelerated_pi_cam/video_system.hpp"

#include <turbojpeg.h>

const int MAX_STREAMS = 9;
const unsigned int CAMERA_WIDTH = 1280;
const unsigned int CAMERA_HEIGHT = 720;

const int CAMERA_UPDATE_INTERVAL = 5000;

const int CAMERA_FRAME_INTERVAL = 1000 / 15;

const int TICK_INTERVAL = 1000;

const int NETWORK_UPDATE_INTERVAL = 1000 / 2;

const int CAMERA_MESSAGE_FRAME_DATA_MAX_SIZE = network::MAX_MESSAGE_SIZE - network::CameraMessage::HEADER_SIZE;

struct Config
{
    int base_station_port;
    int rover_port;
    int video_port;

    char base_station_multicast_group[16];
    char rover_multicast_group[16];
    char video_multicast_group[16];
    char interface[16];
};

class Session{
private:
public:
    util::Clock global_clock;
    Config config;

    unsigned int frame_counter;
    camera::CaptureSession* streams[MAX_STREAMS] = {0};
    VideoSystem accel_video_system;
    bool using_accel_system = false;

    network::Feed r_feed;
    network::Feed bs_feed;
    network::Feed v_feed;

    util::Timer camera_update_timer;
    util::Timer tick_timer;
    util::Timer network_update_timer;

    uint32_t ticks;

    tjhandle compressor;
    tjhandle decompressor;

    unsigned int jpeg_quality;
    bool greyscale;
    network::CameraControlMessage::sendType streamTypes[MAX_STREAMS];

    network::FocusModeMessage::FocusMode video_focus_mode;

    Session();
    ~Session();

    void stderr_handler(logger::Level level, std::string message);
    void load_config(const char* filename);
    int updateCameraStatus();
};

#endif