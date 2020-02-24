#include "../network/network.hpp"
#include "../simple_config/simpleconfig.h"
#include "../util/util.hpp"
#include "../logger/logger.hpp"
#include "../rocs/rocs.hpp"

#include "gps.hpp"
#include "camera.hpp"
#include "subsystem.hpp"
#include "suspension.hpp"
#include "lidar.hpp"

#include <turbojpeg.h>

#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <vector>

// GLOBAL CONSTANTS

// ROCS addresses.
const uint8_t SUSPENSION_I2C_ADDR = 0x01;

const int SUSPENSION_UPDATE_INTERVAL = 1000 / 15;

const int SUSPENSION_CONNECT_TRIES = 5;

const int MAX_STREAMS = 9;
const unsigned int CAMERA_WIDTH = 1280;
const unsigned int CAMERA_HEIGHT = 720;

const int CAMERA_UPDATE_INTERVAL = 5000;

const int CAMERA_FRAME_INTERVAL = 1000 / 15;

const int NETWORK_UPDATE_INTERVAL = 1000 / 2;

const int SUBSYSTEM_SEND_INTERVAL = 1000 * 5;

const int LOCATION_SEND_INTERVAL = 1000;

const int LIDAR_UPDATE_INTERVAL = 1000 / 15;

const int TICK_INTERVAL = 1000;

const int CAMERA_MESSAGE_FRAME_DATA_MAX_SIZE = network::MAX_MESSAGE_SIZE - network::CameraMessage::HEADER_SIZE;

util::Clock global_clock;

struct Config
{
    int base_station_port;
    int rover_port;

    char base_station_multicast_group[16];
    char rover_multicast_group[16];
    char interface[16];

    // For now, this is dynamically-sized.
    char* gps_serial_id;
};

Config load_config(const char* filename) {
    Config config;

    sc::SimpleConfig* sc_config;

    auto err = sc::parse(filename, &sc_config);
    if (err != sc::Error::OK) {
        logger::log(logger::ERROR, "Failed to parse config file: %s", sc::get_error_string(sc_config, err));
        exit(1);
    }

    char* rover_port = sc::get(sc_config, "rover_port");
    if (!rover_port) {
        logger::log(logger::ERROR, "Config file missing 'rover_port'!");
        exit(1);
    }
    config.rover_port = atoi(rover_port);

    char* base_station_port = sc::get(sc_config, "base_station_port");
    if (!base_station_port) {
        logger::log(logger::ERROR, "Config file missing 'base_station_port'!");
        exit(1);
    }
    config.base_station_port = atoi(base_station_port);

    char* base_station_multicast_group = sc::get(sc_config, "base_station_multicast_group");
    if (!base_station_multicast_group) {
        logger::log(logger::ERROR, "Config file missing 'base_station_multicast_group'!");
        exit(1);
    }
    strncpy(config.base_station_multicast_group, base_station_multicast_group, 16);

    char* rover_multicast_group = sc::get(sc_config, "rover_multicast_group");
    if (!rover_multicast_group) {
        logger::log(logger::ERROR, "Config file missing 'rover_multicast_group'!");
        exit(1);
    }
    strncpy(config.rover_multicast_group, rover_multicast_group, 16);

    char* interface = sc::get(sc_config, "interface");
    if (!interface) {
        // Default.
        strncpy(config.interface, "0.0.0.0", 16);
    } else {
        strncpy(config.interface, interface, 16);
    }

    char* gps_serial_id = sc::get(sc_config, "gps_serial_id");
    if (!gps_serial_id) {
        logger::log(logger::ERROR, "Config file missing 'gps_serial_id'!");

        exit(1);
    }
    config.gps_serial_id = strdup(gps_serial_id);

    sc::free(sc_config);

    return config;
}

/**
 * @return: numOpen - the number of open cameras after updating.
 **/
int updateCameraStatus(camera::CaptureSession **streams) {
    /**
     * We need 2 arrays to keep track of all of our data.
     * 1. An array for new cameras found
     * 2. An array for which camera indices still exist
     **/
    int camerasFound[MAX_STREAMS] = {-1};
    int existingCameras[MAX_STREAMS] = {-1};
    int cntr = 0;
    uint8_t open = 1;
    int numOpen = 0;

    for (int i = 0; true; i++) {

        char name_filename_buffer[100];
        sprintf(name_filename_buffer, "/sys/class/video4linux/video%d/name", i);
        FILE* name_file = fopen(name_filename_buffer, "r");

        // We have found a USB file that doesn't exist, therefore no more exist.
        if (!name_file) break;

        fscanf(name_file, "%s\n", name_filename_buffer);
        fclose(name_file);

        if (strncmp("ZED", name_filename_buffer, 3) == 0) continue;

        camerasFound[cntr] = i;
        cntr++;
    }

    /**
     * There are 3 steps here.
     *
     * 1. Check which cameras exist in the file system.
     *
     * 2. Iterate through our cameras adding any extras that do exist.
     *
     * 3. Remove any cameras that don't exist,
     **/

    for(int i = 0; i < cntr; i++) {
        /* 1.  Check which cameras exist in the file system. */
        for(int j = 1; j < MAX_STREAMS; j++) {
            if(streams[j] != nullptr) {
                if(camerasFound[i] == streams[j]->dev_video_id) {
                    /**
                     * Use -1 to say this camera is being used,
                     * so we don't need to do anything.
                     **/
                    camerasFound[i] = -1;
                    existingCameras[j] = i;
                    break;
                }
            }
        }

        /* Don't initialize this camera, as it exists */
        if (camerasFound[i] == -1) {
            continue;
        }
        numOpen++;

        while(streams[open] != nullptr)
            open++;

        char filename_buffer[13]; // "/dev/video" is 10 chars long, leave 2 for numbers, and one for null terminator.
        sprintf(filename_buffer, "/dev/video%d", camerasFound[i]);

        camera::CaptureSession* cs = new camera::CaptureSession;
        logger::log(logger::DEBUG, "Opening camera %d", camerasFound[i]);
        camera::Error err = camera::open(cs, filename_buffer, CAMERA_WIDTH, CAMERA_HEIGHT, camerasFound[i], &global_clock, CAMERA_FRAME_INTERVAL);
        
        if (err != camera::Error::OK) {
            camerasFound[i] = -1;
            logger::log(logger::DEBUG, "Camera %d errored while opening", cs->dev_video_id);
            delete cs;
            continue;
        }

        // Start the camera.
        err = camera::start(cs);
        if (err != camera::Error::OK) {
            camerasFound[i] = -1;
            logger::log(logger::DEBUG, "Camera %d errored while starting", cs->dev_video_id);
            camera::close(cs);
            delete cs;
            continue;
        }

        /** 
          * 2. Iterate through our cameras adding any extras that do exist.
         **/
        streams[open] = cs;
    }

    for(int i = 0; i < cntr; i++) {
        if (camerasFound[i] != -1) {
            logger::log(logger::INFO, "Connected new camera at /dev/video%d", camerasFound[i]);
        }
    }

    /* 3. Remove any cameras that don't exist. */
    for(int j = 1; j < MAX_STREAMS; j++) {
        if(existingCameras[j] == -1 && streams[j] != nullptr) {
            logger::log(logger::INFO, "Camera %d disconnected.", j);
            camera::close(streams[j]);
            delete streams[j];
            streams[j] = nullptr;
        }
    }

    return numOpen;
}

void stderr_handler(logger::Level leve, std::string message) {
    fprintf(stderr, "%s\n", message.c_str());
}

int main() {
    logger::register_handler(stderr_handler);

    util::Clock::init(&global_clock);

    Config config = load_config("res/r.sconfig");

    unsigned int frame_counter = 0;

    // i2c
    if (rocs::init("/dev/i2c-0") != rocs::Error::OK) {
        logger::log(logger::ERROR, "[!] Failed to init ROCS!");
        return 1;
    }

    bool suspension_inited = false;
    for (int i = 0; i < SUSPENSION_CONNECT_TRIES; i++) {
        if (suspension::init(SUSPENSION_I2C_ADDR) != suspension::Error::OK) {
            logger::log(logger::WARNING, "[!] Failed to init suspension (try %d).", i);
        } else {
            suspension_inited = true;
            break;
        }
    }

    if (!suspension_inited) {
        logger::log(logger::ERROR, "[!] Failed to start suspension!");
        return 1;
    }

    if (lidar::start("192.168.1.21") != lidar::Error::OK) {
        logger::log(logger::ERROR, "[!] Failed to start lidar!");
        return 1;
    }

    // Camera streams
    camera::CaptureSession * streams[MAX_STREAMS] = {0};
    int activeCameras = updateCameraStatus(streams);

    if (gps::init(config.gps_serial_id, &global_clock) != gps::Error::OK) {
        logger::log(logger::ERROR, "[!] Failed to open GPS!");
        return 1;
    }

    // Two feeds: incoming base station and outgoing rover.
    network::Feed r_feed, bs_feed;

    {
        auto err = network::init(
            &r_feed,
            network::FeedType::OUT,
            config.interface,
            config.rover_multicast_group,
            config.rover_port,
            &global_clock);

        if (err != network::Error::OK) {
            logger::log(logger::ERROR, "[!] Failed to start rover feed: %s", network::get_error_string(err));
            exit(1);
        }
    }

    {
        auto err = network::init(
            &bs_feed,
            network::FeedType::IN,
            config.interface,
            config.base_station_multicast_group,
            config.base_station_port,
            &global_clock);

        if (err != network::Error::OK) {
            logger::log(logger::ERROR, "[!] Failed to subscribe to base station feed: %s", network::get_error_string(err));
            exit(1);
        }
    }

    util::Timer camera_update_timer;
    util::Timer::init(&camera_update_timer, CAMERA_UPDATE_INTERVAL, &global_clock);

    util::Timer location_send_timer;
    util::Timer::init(&location_send_timer,LOCATION_SEND_INTERVAL, &global_clock);

    util::Timer tick_timer;
    util::Timer::init(&tick_timer, TICK_INTERVAL, &global_clock);

    util::Timer network_update_timer;
    util::Timer::init(&network_update_timer, NETWORK_UPDATE_INTERVAL, &global_clock);

    util::Timer subsystem_send_timer;
    util::Timer::init(&subsystem_send_timer, SUBSYSTEM_SEND_INTERVAL, &global_clock);

    util::Timer suspension_update_timer;
    util::Timer::init(&suspension_update_timer, SUSPENSION_UPDATE_INTERVAL, &global_clock);

    util::Timer lidar_update_timer;
    util::Timer::init(&lidar_update_timer, LIDAR_UPDATE_INTERVAL, &global_clock);

    uint32_t ticks = 0;

    network::MovementMessage last_movement_message = { 0, 0 };

    std::vector<long> lidar_points;

    auto compressor = tjInitCompress();
    auto decompressor = tjInitDecompress();

    unsigned long jpeg_size = tjBufSize(1280, 720, TJSAMP_444);
    uint8_t* jpeg_buffer = (uint8_t*) malloc(jpeg_size);

    // jpeg_quality ranges from 0 - 100, and dictates the level of compression.
    unsigned int jpeg_quality = 30;
    bool greyscale = false;
    network::CameraControlMessage::sendType streamTypes[MAX_STREAMS];
    // Set the starting 2 
    for(int i = 0; i < 2; i++) {
        streamTypes[i] = network::CameraControlMessage::sendType::SEND;
    }
    for(size_t i = 2; i < MAX_STREAMS; i++) {
        streamTypes[i] = network::CameraControlMessage::sendType::DONT_SEND;
    }
    
    while (true) {
        for (size_t i = 1; i < MAX_STREAMS; i++) {
            camera::CaptureSession* cs = streams[i];
            if(cs == nullptr) continue;
            if(streamTypes[i] == network::CameraControlMessage::sendType::DONT_SEND) continue;

            // Grab a frame.
            uint8_t* frame_buffer;
            size_t frame_size;
            {
                camera::Error err = camera::grab_frame(cs, &frame_buffer, &frame_size);
                if (err != camera::Error::OK) {
                    if (err == camera::Error::AGAIN)
                        continue;

                    logger::log(logger::DEBUG, "Deleting camera %d, because it errored", streams[i]->dev_video_id);
                    camera::close(cs);
                    delete cs;
                    streams[i] = nullptr;
                    continue;
                }
            }

            // Decode the frame and encode it again to set our desired quality.
            static uint8_t raw_buffer[CAMERA_WIDTH * CAMERA_HEIGHT * 3];

            // Decompress into a raw frame.

            tjDecompress2(
                decompressor,
                frame_buffer,
                frame_size,
                raw_buffer,
                CAMERA_WIDTH,
                3 * CAMERA_WIDTH,
                CAMERA_HEIGHT,
                TJPF_RGB,
                0);

            // Recompress into jpeg buffer.
            if (greyscale) {
                tjCompress2(
                    compressor,
                    raw_buffer,
                    CAMERA_WIDTH,
                    3 * CAMERA_WIDTH,
                    CAMERA_HEIGHT,
                    TJPF_RGB,
                    &frame_buffer,
                    &frame_size,
                    TJSAMP_GRAY,
                    jpeg_quality,
                    TJFLAG_NOREALLOC);
            } else {
                tjCompress2(
                    compressor,
                    raw_buffer,
                    CAMERA_WIDTH,
                    3 * CAMERA_WIDTH,
                    CAMERA_HEIGHT,
                    TJPF_RGB,
                    &frame_buffer,
                    &frame_size,
                    TJSAMP_420,
                    jpeg_quality,
                    TJFLAG_NOREALLOC);
            }

            /*
                    2. Send out packets
                            - Create camera messages and send Camera packets
            */

            // Send the frame.
            // Calculate how many buffers we will need to send the entire frame
            uint8_t num_buffers = (frame_size / CAMERA_MESSAGE_FRAME_DATA_MAX_SIZE) + 1;

            for (uint8_t j = 0; j < num_buffers; j++) {
                // This accounts for the last buffer that is not completely divisible
                // by the defined buffer size, by using up the remaining space calculated
                // with modulus    -yu
                uint16_t buffer_size = (j != num_buffers - 1) ? CAMERA_MESSAGE_FRAME_DATA_MAX_SIZE :
                                                                frame_size % CAMERA_MESSAGE_FRAME_DATA_MAX_SIZE;

                network::CameraMessage message = {
                    static_cast<uint8_t>(i), // stream_index
                    static_cast<uint16_t>(frame_counter), // frame_index
                    static_cast<uint8_t>(j), // section_index
                    num_buffers, // section_count
                    buffer_size, // size
                    frame_buffer + (CAMERA_MESSAGE_FRAME_DATA_MAX_SIZE * j) // data
                };

                network::publish(&r_feed, &message);
            }

            camera::return_buffer(cs);
        }

        if (camera_update_timer.ready()) {
            updateCameraStatus(streams);
        }

        if (location_send_timer.ready()) {
            network::LocationMessage message;

            auto fix = gps::get_fix();
            message.fix_status = fix;

            auto pos = gps::get_position();
            message.latitude = pos.latitude;
            message.longitude = pos.longitude;


            network::publish(&r_feed,&message);
        }

        if (lidar_update_timer.ready()) {
            network::LidarMessage message;

            lidar_points.clear();
            if (lidar::scan(lidar_points) != lidar::Error::OK) {
                logger::log(logger::ERROR, "Lidar scan failed");
            } else {
                for (int i = 0; i < network::LidarMessage::NUM_LIDAR_POINTS; i++) {
                    message.points[i] = (uint16_t) lidar_points[i];
                }

                network::publish(&r_feed, &message);
            }
        }

        // Increment global (across all streams) frame counter. Should be ok. Should...
        frame_counter++;

        // Receive incoming messages
        network::IncomingMessage message;

        while (true) {
            auto neterr = network::receive(&bs_feed, &message);
            if (neterr != network::Error::OK) {
                if (neterr == network::Error::NOMORE) {
                    break;
                }

                logger::log(logger::DEBUG, "Network error on receive!");

                break;
            }
            switch (message.type) {
                case network::MessageType::MOVEMENT: {
                    network::deserialize(&message.buffer, &last_movement_message);

                    break;
                }
                case network::MessageType::CAMERA_CONTROL: {
                    network::CameraControlMessage quality;
                    network::deserialize(&message.buffer, &quality);
                    network::CameraControlMessage::Setting setting = quality.setting;

                    switch(setting) {
                        case network::CameraControlMessage::Setting::JPEG_QUALITY:
                            jpeg_quality = quality.jpegQuality;
                            break;
                        case network::CameraControlMessage::Setting::GREYSCALE:
                            greyscale = quality.greyscale;
                            break;
                        case network::CameraControlMessage::Setting::DISPLAY_STATE:
                            streamTypes[quality.resolution.stream_index] = quality.resolution.sending;
                            break;
                    }
                    break;
                }
                case network::MessageType::ARM: {
                    network::ArmMessage arm_message;
                    network::deserialize(&message.buffer, &arm_message);

                    /*
                    logger::log(logger::DEBUG, "arm update:");
                    logger::log(logger::DEBUG, "  gfinger=%d", static_cast<uint8_t>(arm_message.get_state(network::ArmMessage::Motor::GRIPPER_FINGER)));
                    logger::log(logger::DEBUG, "  gwrotate=%d", static_cast<uint8_t>(arm_message.get_state(network::ArmMessage::Motor::GRIPPER_WRIST_ROTATE)));
                    logger::log(logger::DEBUG, "  gwflex=%d", static_cast<uint8_t>(arm_message.get_state(network::ArmMessage::Motor::GRIPPER_WRIST_FLEX)));
                    logger::log(logger::DEBUG, "  arml=%d", static_cast<uint8_t>(arm_message.get_state(network::ArmMessage::Motor::ARM_LOWER)));
                    logger::log(logger::DEBUG, "  armu=%d", static_cast<uint8_t>(arm_message.get_state(network::ArmMessage::Motor::ARM_UPPER)));
                    logger::log(logger::DEBUG, "  armb=%d", static_cast<uint8_t>(arm_message.get_state(network::ArmMessage::Motor::ARM_BASE)));
                    */

                    break;
                }
                default:
                    break;
            }
        }

        if (network_update_timer.ready()) {
            // Update feed statuses.
            network::update_status(&r_feed);
            network::update_status(&bs_feed);
        }

        if (suspension_update_timer.ready()) {
            auto movement = last_movement_message;

            suspension::Direction left_dir;
            suspension::Direction right_dir;

            if (movement.left < 0) {
                left_dir = suspension::Direction::BACKWARD;
                movement.left = -movement.left;
            } else if (movement.left > 0) {
                left_dir = suspension::Direction::FORWARD;
            }

            if (movement.left > 255) movement.left = 255;

            if (movement.right < 0) {
                right_dir = suspension::Direction::BACKWARD;
                movement.right = -movement.right;
            } else if (movement.right > 0) {
                right_dir = suspension::Direction::FORWARD;
            } 

            if (movement.right > 255) movement.right = 255;

            if (movement.left == 0) {
                if (suspension::stop(suspension::Side::LEFT) != suspension::Error::OK) {
                    logger::log(logger::ERROR, "Failed to stop left suspension side");
                }
            } else {
                if (suspension::update(suspension::Side::LEFT, left_dir, (uint8_t)movement.left) != suspension::Error::OK) {

                    logger::log(logger::ERROR, "Failed to update left suspension side");
                }
            }

            if (movement.right == 0) {
                if (suspension::stop(suspension::Side::RIGHT) != suspension::Error::OK) {
                    logger::log(logger::ERROR, "Failed to stop right suspension side");
                }
            } else {
                if (suspension::update(suspension::Side::RIGHT, right_dir, (uint8_t)movement.right) != suspension::Error::OK) {

                    logger::log(logger::ERROR, "Failed to update right suspension side");
                }
            }
        }

        // Tick.
        ticks++;
        uint32_t last_tick_interval;
        if (tick_timer.ready(&last_tick_interval)) {
            network::TickMessage message;
            message.ticks_per_second = (ticks * TICK_INTERVAL) / (float)last_tick_interval;
            network::publish(&r_feed, &message);

            ticks = 0;
        }

        if (subsystem_send_timer.ready()) {
            subsystem::send_update(&r_feed);
        }
    }
}
