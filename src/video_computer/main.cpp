#include "session.hpp"

int main(){
    Session video_session;
    logger::register_handler(logger::stderr_handler);
    util::Clock::init(&video_session.global_clock);
    video_session.config = video_session.load_config("res/r.sconfig");
    video_session.updateCameraStatus();

    // Two feeds: incoming base station and outgoing rover.
    {
        auto err = network::init(
            &video_session.r_feed,
            network::FeedType::OUT,
            video_session.config.interface,
            video_session.config.rover_multicast_group,
            video_session.config.rover_port,
            &video_session.global_clock);
        if (err != network::Error::OK) {
            logger::log(logger::ERROR, "[!] Failed to start rover feed: %s", network::get_error_string(err));
            exit(1);
        }
    }
    {
        auto err = network::init(
            &video_session.bs_feed,
            network::FeedType::IN,
            video_session.config.interface,
            video_session.config.base_station_multicast_group,
            video_session.config.base_station_port,
            &video_session.global_clock);
        if (err != network::Error::OK) {
            logger::log(logger::ERROR, "[!] Failed to subscribe to base station feed: %s", network::get_error_string(err));
            exit(1);
        }
    }

    util::Timer::init(&video_session.camera_update_timer, CAMERA_UPDATE_INTERVAL, &video_session.global_clock);
    util::Timer::init(&video_session.tick_timer, TICK_INTERVAL, &video_session.global_clock);
    util::Timer::init(&video_session.network_update_timer, NETWORK_UPDATE_INTERVAL, &video_session.global_clock);
    
    // Set the starting 2 
    for(int i = 0; i < 2; i++) {
        //streamTypes[i] = network::CameraControlMessage::sendType::SEND;
        video_session.streamTypes[i] = network::CameraControlMessage::sendType::SEND;
    }
    for(size_t i = 2; i < MAX_STREAMS; i++) {
        //streamTypes[i] = network::CameraControlMessage::sendType::DONT_SEND;
        video_session.streamTypes[i] = network::CameraControlMessage::sendType::DONT_SEND;
    }
    while (true) {
        for (size_t i = 1; i < MAX_STREAMS; i++) {
            camera::CaptureSession* cs = video_session.streams[i];
            if(cs == nullptr) continue;
            if(video_session.streamTypes[i] == network::CameraControlMessage::sendType::DONT_SEND) continue;
            // Grab a frame.
            uint8_t* frame_buffer;
            size_t frame_size;
            {
                camera::Error err = camera::grab_frame(cs, &frame_buffer, &frame_size);
                if (err != camera::Error::OK) {
                    if (err == camera::Error::AGAIN)
                        continue;

                    logger::log(logger::DEBUG, "Deleting camera %d, because it errored", video_session.streams[i]->dev_video_id);
                    camera::close(cs);
                    delete cs;
                    video_session.streams[i] = nullptr;
                    continue;
                }
            }
            long unsigned int long_frame_size = frame_size;
            // Decode the frame and encode it again to set our desired quality.
            static uint8_t raw_buffer[CAMERA_WIDTH * CAMERA_HEIGHT * 3];
            // Decompress into a raw frame.
            tjDecompress2(
                video_session.decompressor,
                frame_buffer,
                frame_size,
                raw_buffer,
                CAMERA_WIDTH,
                3 * CAMERA_WIDTH,
                CAMERA_HEIGHT,
                TJPF_RGB,
                0);
            // Recompress into jpeg buffer.
            if (video_session.greyscale) {
                tjCompress2(
                    video_session.compressor,
                    raw_buffer,
                    CAMERA_WIDTH,
                    3 * CAMERA_WIDTH,
                    CAMERA_HEIGHT,
                    TJPF_RGB,
                    &frame_buffer,
                    &long_frame_size,
                    TJSAMP_GRAY,
                    video_session.jpeg_quality,
                    TJFLAG_NOREALLOC);
            } else {
                tjCompress2(
                    video_session.compressor,
                    raw_buffer,
                    CAMERA_WIDTH,
                    3 * CAMERA_WIDTH,
                    CAMERA_HEIGHT,
                    TJPF_RGB,
                    &frame_buffer,
                    &long_frame_size,
                    TJSAMP_420,
                    video_session.jpeg_quality,
                    TJFLAG_NOREALLOC);
            }
            // 2. Send out packets
            //   - Create camera messages and send Camera packets
            //
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
                    static_cast<uint16_t>(video_session.frame_counter), // frame_index
                    static_cast<uint8_t>(j), // section_index
                    num_buffers, // section_count
                    buffer_size, // size
                    frame_buffer + (CAMERA_MESSAGE_FRAME_DATA_MAX_SIZE * j) // data
                };
                network::publish(&video_session.r_feed, &message);
            }
            camera::return_buffer(cs);
        }

        if (video_session.camera_update_timer.ready()) {
            video_session.updateCameraStatus();
        }

        // Increment global (across all streams) frame counter. Should be ok. Should...
        video_session.frame_counter++;

        // Receive incoming messages
        network::IncomingMessage message;

        while (true) {
            auto neterr = network::receive(&video_session.bs_feed, &message);
            if (neterr != network::Error::OK) {
                if (neterr == network::Error::NOMORE) {
                    break;
                }
                logger::log(logger::DEBUG, "Network error on receive!");
                break;
            }
            switch (message.type) {
                case network::MessageType::CAMERA_CONTROL: {
                    network::CameraControlMessage quality;
                    network::deserialize(&message.buffer, &quality);
                    network::CameraControlMessage::Setting setting = quality.setting;
                    switch(setting) {
                        case network::CameraControlMessage::Setting::JPEG_QUALITY:
                            video_session.jpeg_quality = quality.jpegQuality;
                            break;
                        case network::CameraControlMessage::Setting::GREYSCALE:
                            video_session.greyscale = quality.greyscale;
                            break;
                        case network::CameraControlMessage::Setting::DISPLAY_STATE:
                            video_session.streamTypes[quality.resolution.stream_index] = quality.resolution.sending;
                            break;
                    }
                    break;
                }
                default:
                    break;
            }
        }
        if (video_session.network_update_timer.ready()) {
            // Update feed statuses.
            network::update_status(&video_session.r_feed);
            network::update_status(&video_session.bs_feed);
        }
        // Tick.
        //ticks++;
        video_session.ticks++;
        uint32_t last_tick_interval;
        if (video_session.tick_timer.ready(&last_tick_interval)) {
            network::TickMessage message;
            message.ticks_per_second = (video_session.ticks * TICK_INTERVAL) / (float)last_tick_interval;
            network::publish(&video_session.r_feed, &message);
            video_session.ticks = 0;
        }
    }
}