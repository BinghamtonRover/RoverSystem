#include "session.hpp"

#include "../cpnetwork/stream.hpp"

int main(){
    Session video_session;
    logger::register_handler(logger::stderr_handler);
    util::Clock::init(&video_session.global_clock);
    video_session.load_config("res/v.sconfig");
    video_session.updateCameraStatus();

    boost::asio::io_context ctx;
    net::StreamSender video_sender(ctx);
    video_sender.set_destination_endpoint(boost::asio::ip::udp::endpoint(boost::asio::ip::address_v4::from_string("127.0.0.1"), 1500));
    video_sender.create_streams(MAX_STREAMS);

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
            video_sender.send_frame(i, frame_buffer, long_frame_size);
            camera::return_buffer(cs);
        }

        if (video_session.camera_update_timer.ready()) {
            video_session.updateCameraStatus();
        }

        // Increment global (across all streams) frame counter. Should be ok. Should...
        video_session.frame_counter++;
        
        // Receive incoming messages
        //network::IncomingMessage message;
        //network::receive(&video_session.bs_feed, &message);

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