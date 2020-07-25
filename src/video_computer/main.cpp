#include "../network/network.hpp"
#include "../logger/logger.hpp"

#include "camera.hpp"

#include <turbojpeg.h>
#include <cstring>

const int MAX_STREAMS = 9;
const unsigned int CAMERA_WIDTH = 1280;
const unsigned int CAMERA_HEIGHT = 720;

const int CAMERA_UPDATE_INTERVAL = 5000;

const int CAMERA_FRAME_INTERVAL = 1000 / 15;

util::Clock global_clock;

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

int main(){
    // Camera streams
    camera::CaptureSession * streams[MAX_STREAMS] = {0};
    updateCameraStatus(streams);
}