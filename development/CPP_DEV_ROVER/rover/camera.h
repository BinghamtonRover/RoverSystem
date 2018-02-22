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

namespace camera {

// The number of buffers to use while capturing frames.
// Since each buffer corresponds to a single frame, this value represents
// the maximum number of frames which can be read at a time.
const int NUM_BUFFERS = 4;

// The video format we accept. This is pretty standard, but some
// webcams may use a different one. We need to make sure that
// the ones we use will support this. If not, it is not so hard
// to change later.
const uint32_t PIXEL_FORMAT = V4L2_PIX_FMT_MJPEG;

// The timeout used when waiting for frames, in whole seconds.
// TODO: Make this more reasonable.
const int SELECT_TIMEOUT = 2;

// Represents a buffer that has a size.
struct Buffer {
    size_t size;
    uint8_t* data;

    Buffer():
        data(nullptr) {}

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

    // Creates a session with the desired width and height.
    // Does not open a specific camera.
    CaptureSession(size_t _width, size_t _height):
        fd(-1), width(_width), height(_height) {}

    // Closes the connection to the camera.
    ~CaptureSession();

    // Opens a capture session with the camera located at the given file path.
    // Returns true on success and false on failure.
    bool open(std::string camera_path);

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

    // Waits for the next frame from the camera and copies the JPEG frame
    // into the given buffer, which must have size ''image_size''.
    // Method ''start_stream'' MUST be called first. Returns the size of
    // the frame on success and 0 otherwise.
    size_t grab_frame(uint8_t* buffer);
};

} // namespace camera