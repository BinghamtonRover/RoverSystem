#include "../network/network.hpp"
#include "../simple_config/simpleconfig.h"
#include "../util/util.hpp"

#include "gps.hpp"
#include "camera.hpp"
#include "zed.hpp"
#include "subsystem.hpp"

#include <turbojpeg.h>

#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <vector>

// GLOBAL CONSTANTS

const int MAX_STREAMS = 9;
const unsigned int CAMERA_WIDTH = 1280;
const unsigned int CAMERA_HEIGHT = 720;

const int CAMERA_UPDATE_INTERVAL = 5000;

const int CAMERA_FRAME_INTERVAL = 1000 / 15;

const int NETWORK_UPDATE_INTERVAL = 1000 / 2;

const int SUBSYSTEM_SEND_INTERVAL = 1000 * 5;

const int LOCATION_SEND_INTERVAL = 1000;

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
};

Config load_config(const char* filename) {
    Config config;

    sc::SimpleConfig* sc_config;

    auto err = sc::parse(filename, &sc_config);
    if (err != sc::Error::OK) {
        printf("Failed to parse config file: %s\n", sc::get_error_string(sc_config, err));
        exit(1);
    }

    char* rover_port = sc::get(sc_config, "rover_port");
    if (!rover_port) {
        printf("Config file missing 'rover_port'!\n");
        exit(1);
    }
    config.rover_port = atoi(rover_port);

    char* base_station_port = sc::get(sc_config, "base_station_port");
    if (!base_station_port) {
        printf("Config file missing 'base_station_port'!\n");
        exit(1);
    }
    config.base_station_port = atoi(base_station_port);

    char* base_station_multicast_group = sc::get(sc_config, "base_station_multicast_group");
    if (!base_station_multicast_group) {
        printf("Config file missing 'base_station_multicast_group'!\n");
        exit(1);
    }
    strncpy(config.base_station_multicast_group, base_station_multicast_group, 16);

    char* rover_multicast_group = sc::get(sc_config, "rover_multicast_group");
    if (!rover_multicast_group) {
        printf("Config file missing 'rover_multicast_group'!\n");
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
        printf("Opening camera %d\n", camerasFound[i]);
        camera::Error err = camera::open(cs, filename_buffer, CAMERA_WIDTH, CAMERA_HEIGHT, camerasFound[i], &global_clock, CAMERA_FRAME_INTERVAL);
        
        if (err != camera::Error::OK) {
            camerasFound[i] = -1;
            printf("> Camera %d errored while opening\n", cs->dev_video_id);
            delete cs;
            continue;
        }

        // Start the camera.
        err = camera::start(cs);
        if (err != camera::Error::OK) {
            camerasFound[i] = -1;
            printf("> Camera %d errored while starting\n", cs->dev_video_id);
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
            printf("> Connected new camera at /dev/video%d\n", camerasFound[i]);
        }
    }

    /* 3. Remove any cameras that don't exist. */
    for(int j = 1; j < MAX_STREAMS; j++) {
        if(existingCameras[j] == -1 && streams[j] != nullptr) {
            printf("Deleting camera %d.\n", j);
            camera::close(streams[j]);
            delete streams[j];
            streams[j] = nullptr;
        }
    }

    return numOpen;
}

int main() {
    util::Clock::init(&global_clock);

    Config config = load_config("res/r.sconfig");

    unsigned int frame_counter = 0;

    // Camera streams
    camera::CaptureSession * streams[MAX_STREAMS] = {0};
    int activeCameras = updateCameraStatus(streams);

    // Note, this doesn't count the zed camera.
    printf("> Started with %d cameras conncted.\n", activeCameras);

    //std::cout << "> Using " << streams.size() << " cameras." << std::endl;

    if (zed::open(&global_clock) != zed::Error::OK) {
        printf("> Failed to open zed camera!\n");
        return 1;
    }

    if (gps::open() != gps::Error::OK) {
        printf("[!] Failed to open GPS!\n");
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
            printf("[!] Failed to start rover feed: %s\n", network::get_error_string(err));
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
            printf("[!] Failed to subscribe to base station feed: %s\n", network::get_error_string(err));
            printf("errno %d\n", errno);
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

    uint32_t ticks = 0;

    auto compressor = tjInitCompress();
    auto decompressor = tjInitDecompress();


    unsigned long jpeg_size = tjBufSize(1280, 720, TJSAMP_444);
    uint8_t* jpeg_buffer = (uint8_t*) malloc(jpeg_size);

    // jpeg_quality ranges from 0 - 100, and dictates the level of compression.
    unsigned int jpeg_quality = 30;
    bool greyscale = false;
    while (true) {
        for (size_t i = 0; i < MAX_STREAMS; i++) {
            camera::CaptureSession* cs = streams[i];
            if(cs == nullptr) continue;

            // Grab a frame.
            uint8_t* frame_buffer;
            size_t frame_size;
            {
                camera::Error err = camera::grab_frame(cs, &frame_buffer, &frame_size);
                if (err != camera::Error::OK) {
                    if (err == camera::Error::AGAIN)
                        continue;

                    printf("Deleting camera %d, because it errored\n", streams[i]->dev_video_id);
                    camera::close(cs);
                    delete cs;
                    streams[i] = nullptr;
                    continue;
                    //std::cout << "Camera 0: " << camera::get_error_string(err) << std::endl;
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

            bool fix = gps::has_fix();
            message.has_fix = fix;

            if (fix) {
                auto pos = gps::get_current_position();
                message.latitude = pos.latitude;
                message.longitude = pos.longitude;
            }

            network::publish(&r_feed,&message);
        }

        unsigned char* zed_image;
        int zed_stride;
        zed::Pose zed_pose;
        if (zed::grab(&zed_image, &zed_stride, &zed_pose) == zed::Error::OK) {
            jpeg_size = tjBufSize(1280, 720, TJSAMP_422);

            auto tj_err = tjCompress2(
                compressor,
                zed_image,
                1280,
                zed_stride,
                720,
                TJPF_BGRA,
                (unsigned char**) &jpeg_buffer,
                &jpeg_size,
                TJSAMP_422,
                40,
                TJFLAG_NOREALLOC);

            /*
                        auto tj_err = tjCompressFromYUV(compressor, zed_image.getPtr<unsigned char>(), 1280,
                                                        zed_image.getStepBytes(), 720, TJSAMP_444, &jpeg_buffer,
                                                        &jpeg_size, 40, TJFLAG_NOREALLOC);
            */
            if (tj_err != 0) {
                fprintf(stderr, "[!] tjCompress failed: %s\n", tjGetErrorStr2(compressor));
            }

            // Send the frame.
            // Calculate how many buffers we will need to send the entire frame
            uint8_t num_buffers = (jpeg_size / CAMERA_MESSAGE_FRAME_DATA_MAX_SIZE) + 1;

            for (uint8_t j = 0; j < num_buffers; j++) {
                // This accounts for the last buffer that is not completely divisible
                // by the defined buffer size, by using up the remaining space calculated
                // with modulus    -yu
                uint16_t buffer_size = (j != num_buffers - 1) ? CAMERA_MESSAGE_FRAME_DATA_MAX_SIZE :
                                                                jpeg_size % CAMERA_MESSAGE_FRAME_DATA_MAX_SIZE;

                network::CameraMessage message = {
                    static_cast<uint8_t>(0), // stream_index
                    static_cast<uint16_t>(frame_counter), // frame_index
                    static_cast<uint8_t>(j), // section_index
                    num_buffers, // section_count
                    buffer_size, // size
                    jpeg_buffer + (CAMERA_MESSAGE_FRAME_DATA_MAX_SIZE * j) // data
                };

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

                printf("[!] Network error on receive!\n");

                break;
            }
            switch (message.type) {
                case network::MessageType::MOVEMENT: {
                    network::MovementMessage movement;
                    network::deserialize(&message.buffer, &movement);

					// printf("Got movement with %d, %d\n", movement.left, movement.right);
                    // TODO: Feed suspension here!

                    break;
                }
                case network::MessageType::JPEGQUALITY: {
                    network::JpegQualityMessage quality;
                    network::deserialize(&message.buffer, &quality);
                    uint8_t setting = quality.setting;
                    if (setting == 0){
                        jpeg_quality = quality.jpegQuality;
                        
                    }
                    else if (setting == 1){
                        greyscale = quality.greyscale;
                    }
                    
                    
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
