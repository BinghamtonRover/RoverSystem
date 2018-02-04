#include <stdio.h>
#include <linux/videodev2.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <sys/select.h>
#include <stdint.h>

#include <string>

#include <opencv2/opencv.hpp>

// The number of buffers to use while capturing frames.
// Since each buffer corresponds to a single frame, this value represents
// the maximum number of frames which can be read at a time.
#define NUM_BUFFERS 4

// The video format we accept. This is pretty standard, but some
// webcams may use a different one. We need to make sure that
// the ones we use will support this. If not, it is not so hard
// to change later.
// In this case, the pixel format here is described at the following location:
// https://linuxtv.org/downloads/v4l-dvb-apis/uapi/v4l/pixfmt-yuyv.html.
// The colors are defined in the YCbCr format. A to-RGB conversion routine
// can be found below.
#define PIXEL_FORMAT V4L2_PIX_FMT_YUYV

// The timeout used when waiting for frames, in whole seconds.
#define SELECT_TIMEOUT 2

// Represents a buffer that has a size.
struct Buffer {
    size_t size;
    uint8_t* data;

    // Allocates the buffer. Only call this once!
    void allocate(size_t _size);

    // Frees this buffer's data.
    ~Buffer();
};

// Represents an open capture session for a specific camera.
// Contains all buffers, file descriptors, and other information
// corresponding to the session.
struct CaptureSession {
    // The file descriptor of the open camera.
    int fd;

    // The width and height of the camera.
    size_t width, height;
    // The size of each frame, in bytes.
    size_t image_size;

    // An array of buffers to use when reading from the camera.
    Buffer buffers[NUM_BUFFERS];

    // Closes the connection to the camera.
    ~CaptureSession();

    // Starts a capture session with the camera located at the given file path.
    // Returns true on success and false on failure.
    bool start(std::string camera_path);

    // Checks the capabilities of the camera to make sure that it supports
    // reading with the default format. Method ''start'' MUST be called first.
    // Also queries the width, height, and frame size (in bytes) of the camera.
    // Returns true if all neccessary capabilities are found and false otherwise.
    bool check_capabilities();

    // Initializes the buffers used to capture frame data.
    // Method ''check_capabilities`` MUST be called first.
    // Returns true on success and false otherwise.
    bool init_buffers();

    // Starts the video stream for this session. Method ''init_buffers''
    // MUST be called first. Returns true on success and false otherwise.
    bool start_stream();

    // Waits for the next frame from the camera, converts each pixel to RGB,
    // and copies the frame to the given OpenCV Mat. Method ''start_stream''
    // MUST be called first. Returns true on success and false otherwise.
    bool grab_frame(cv::Mat& out_image);
};