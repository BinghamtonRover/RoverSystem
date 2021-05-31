#include "session.hpp"
#include "accelerated_pi_cam/video_system_exception.hpp"

int main(){
    Session video_session;
    logger::register_handler(logger::stderr_handler);
    util::Clock::init(&video_session.global_clock);
    video_session.load_config("res/v.sconfig");

    // Must start the accelerated video system first or the Pi camera is taken by the standard video system
    try {
        video_session.accel_video_system.init();
        video_session.using_accel_system = true;
    } catch (const VideoSystemException& video_system_exception) {
        logger::log(logger::DEBUG, "Error opening CSI-attached camera: %s. Accelerated video system will be disabled.", video_system_exception.get_details().c_str());
        video_session.using_accel_system = false;
    }
    video_session.updateCameraStatus();

    //Three feeds: base station, subsystem computer, and video computer.
    {
        auto err = network::init(
            &video_session.r_feed,
            network::FeedType::IN,
            video_session.config.interface,
            video_session.config.rover_multicast_group,
            video_session.config.rover_port,
            &video_session.global_clock);
        if (err != network::Error::OK) {
            logger::log(logger::ERROR, "[!] Failed to subscribe to subsystems feed: %s", network::get_error_string(err));
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
    {
        auto err = network::init(
            &video_session.v_feed,
            network::FeedType::OUT,
            video_session.config.interface,
            video_session.config.video_multicast_group,
            video_session.config.video_port,
            &video_session.global_clock);
        if (err != network::Error::OK) {
            logger::log(logger::ERROR, "[!] Failed to start video feed: %s", network::get_error_string(err));
            exit(1);
        }
    }

    if (video_session.using_accel_system)
        video_session.accel_video_system.start_video();

    util::Timer::init(&video_session.camera_update_timer, CAMERA_UPDATE_INTERVAL, &video_session.global_clock);
    util::Timer::init(&video_session.tick_timer, TICK_INTERVAL, &video_session.global_clock);
    util::Timer::init(&video_session.network_update_timer, NETWORK_UPDATE_INTERVAL, &video_session.global_clock);
    
    // Set the starting 2 
    for(int i = 0; i < 2; i++) {
        video_session.streamTypes[i] = network::CameraControlMessage::sendType::SEND;
    }
    for(size_t i = 2; i < MAX_STREAMS; i++) {
        video_session.streamTypes[i] = network::CameraControlMessage::sendType::DONT_SEND;
    }
    while (true) {

        if (video_session.using_accel_system)
            video_session.accel_video_system.fill_partial_frame_async();

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
                network::publish(&video_session.v_feed, &message);
            }
            camera::return_buffer(cs);
        }

        if (video_session.camera_update_timer.ready()) {
            video_session.updateCameraStatus();
        }

        // Increment global (across all streams) frame counter. Should be ok. Should...
        video_session.frame_counter++;

        // Camera streams using accelerated CSI-attached cameras
        static VideoSystem::MessageBuilder csi_message_builder;

        if (video_session.using_accel_system && video_session.accel_video_system.partial_frame_available_async()) {
            OMX_BUFFERHEADERTYPE *frame;
            network::CSICameraMessage csi_message;

            bool starting_new_frame = video_session.accel_video_system.get_partial_frame_async(&frame);
            if (starting_new_frame) {
                if (csi_message_builder.get_frame_delimiter(csi_message)) {
                    network::publish(&video_session.v_feed, &csi_message);
                }
            }

            csi_message_builder.start_partial_frame(frame);
            while (csi_message_builder.available()) {
                csi_message_builder.fill_message(csi_message);
                network::publish(&video_session.v_feed, &csi_message);
            }
        }

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
                case network::MessageType::FOCUS_MODE: {
                    network::FocusModeMessage focus_mode_message;
                    network::deserialize(&message.buffer, &focus_mode_message);
                    video_session.video_focus_mode = focus_mode_message.focus_mode;
                    // Echo back to BS to verify mode change.
                    network::publish(&video_session.v_feed, &focus_mode_message);
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
            network::update_status(&video_session.v_feed);
        }
        // Tick.
        video_session.ticks++;
        uint32_t last_tick_interval;
        if (video_session.tick_timer.ready(&last_tick_interval)) {
            network::TickMessage message;
            message.ticks_per_second = (video_session.ticks * TICK_INTERVAL) / (float)last_tick_interval;
            network::publish(&video_session.v_feed, &message);
            video_session.ticks = 0;
        }
    }
}