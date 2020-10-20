#include "session.hpp"

#include <cstring>

Session::Session(){
    this->frame_counter = 0;
    this->ticks = 0;
    this->compressor = tjInitCompress();
    this->decompressor = tjInitDecompress();
    this->jpeg_quality = 30;
    this->greyscale = false;
}

Session::~Session() {}

void Session::stderr_handler(logger::Level leve, std::string message) {
    fprintf(stderr, "%s\n", message.c_str());
}

void Session::load_config(const char* filename) {
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
    this->config.rover_port = atoi(rover_port);

    char* base_station_port = sc::get(sc_config, "base_station_port");
    if (!base_station_port) {
        logger::log(logger::ERROR, "Config file missing 'base_station_port'!");
        exit(1);
    }
    this->config.base_station_port = atoi(base_station_port);

    char* video_port = sc::get(sc_config, "video_port");
    if (!video_port) {
        logger::log(logger::ERROR, "Config file missing 'video_port'!");
        exit(1);
    }
    this->config.video_port = atoi(video_port);

    char* base_station_multicast_group = sc::get(sc_config, "base_station_multicast_group");
    if (!base_station_multicast_group) {
        logger::log(logger::ERROR, "Config file missing 'base_station_multicast_group'!");
        exit(1);
    }
    strncpy(this->config.base_station_multicast_group, base_station_multicast_group, 16);

    char* rover_multicast_group = sc::get(sc_config, "rover_multicast_group");
    if (!rover_multicast_group) {
        logger::log(logger::ERROR, "Config file missing 'rover_multicast_group'!");
        exit(1);
    }
    strncpy(this->config.rover_multicast_group, rover_multicast_group, 16);

    char* video_multicast_group = sc::get(sc_config, "video_multicast_group");
    if (!video_multicast_group) {
        logger::log(logger::ERROR, "Config file missing 'video_multicast_group'!");
        exit(1);
    }
    strncpy(this->config.video_multicast_group, video_multicast_group, 16);

    char* interface = sc::get(sc_config, "interface");
    if (!interface) {
        // Default.
        strncpy(this->config.interface, "0.0.0.0", 16);
    } else {
        strncpy(this->config.interface, interface, 16);
    }

    sc::free(sc_config);
}

int Session::updateCameraStatus() {
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
        if (!name_file){
            break;
        } 
        fscanf(name_file, "%s\n", name_filename_buffer);
        fclose(name_file);
        if (strncmp("ZED", name_filename_buffer, 3) == 0){
            continue;
        }
        camerasFound[cntr] = i;
        cntr++;
    }
    // There are 3 steps here.
    // 1. Check which cameras exist in the file system.
    // 2. Iterate through our cameras adding any extras that do exist.
    // 3. Remove any cameras that don't exist,
    for(int i = 0; i < cntr; i++) {
        // 1.  Check which cameras exist in the file system.
        for(int j = 1; j < MAX_STREAMS; j++) {
            if(this->streams[j] != nullptr) { 
                if(camerasFound[i] == this->streams[j]->dev_video_id) {
                    // Use -1 to say this camera is being used,
                    // so we don't need to do anything.
                    camerasFound[i] = -1;
                    existingCameras[j] = i;
                    break;
                }
            }
        }
        // Don't initialize this camera, as it exists 
        if (camerasFound[i] == -1) {
            continue;
        }
        numOpen++;

        while(this->streams[open] != nullptr)
            open++;

        // "/dev/video" is 10 chars long, leave 2 for numbers, and one for null terminator.
        char filename_buffer[13]; 
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
        // 2. Iterate through our cameras adding any extras that do exist.
        this->streams[open] = cs;
    }
    for(int i = 0; i < cntr; i++) {
        if (camerasFound[i] != -1) {
            logger::log(logger::INFO, "Connected new camera at /dev/video%d", camerasFound[i]);
        }
    }
    // 3. Remove any cameras that don't exist.
    for(int j = 1; j < MAX_STREAMS; j++) {
        if(existingCameras[j] == -1 && this->streams[j] != nullptr) {
            logger::log(logger::INFO, "Camera %d disconnected.", j);
            camera::close(this->streams[j]);
            delete this->streams[j];
            this->streams[j] = nullptr;
        }
    }
    return numOpen;
}


