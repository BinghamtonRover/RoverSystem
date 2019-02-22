#ifndef CAMERA_H
#define CAMERA_H

#include <stdint.h>
#include <cstddef>

#include <linux/videodev2.h>

/*
    This library uses the Video4Linux kernel library to grab camera frames.

    Here is a usage example:
    ```
        int main() {
            // Allocate a CameraSession.
            // No fields have to be set.
            camera::CameraSession session;

            // Call open to initialize the session.
            auto err = camera::open(&session, "/dev/video0", 1920, 1080);
            // ... handle error ...

            // Call start to start the stream.
            err = camera::start(&session);
            // ... handle error ...

            while (true) {
                // Grab a frame.
                uint8_t* frame_buffer;
                size_t frame_size;
                err = camera::grab_frame(&session, &frame_buffer, &frame_size);
                // ... handle error ...

                // Return the buffer.
                err = camera::return_buffer(&session, frame_buffer);
                // ... handle error ...

                if (program_should_close) break;
            }

            // Close the session at the end.
            camera::close(&session);
        }
    ```
*/

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

    // A buffer to use when capturing frames.
    uint8_t* frame_buffer;

    // The last buffer we used to capture a frame.
    struct v4l2_buffer last_capture_buffer;

    // The name of the camera.
    char name[32];

    // A string representing the camera's location on the hardware.
    // This is useful for distinguishing cameras, since it will not change
    // unless a camera is moved to a different port on the machine.
    char hardware_location[32];
};

/*
    Macro used to define the error enum values, using "X macros".
    This allows us to define the names only once, and establish an enum
    for the errors as well as string values for easy printing/logging.
*/
#define ERROR_DEF(X) \
    X(OK), \
    X(OPEN), \
    X(QUERY_CAPABILITIES), \
    X(NO_VIDEOCAPTURE), \
    X(NO_STREAMING), \
    X(QUERY_FORMAT), \
    X(SET_FORMAT), \
    X(UNSUPPORTED_FORMAT), \
    X(UNSUPPORTED_RESOLUTION), \
    X(REQUEST_BUFFERS), \
    X(LINK_BUFFERS), \
    X(START_STREAM), \
    X(SELECT), \
    X(READ_FRAME), \
    X(PREPARE_BUFFER)

/*
    Enum definition for the error values.
*/
#define X(name) name
enum class Error {
    ERROR_DEF(X)
};
#undef X

/*
    Returns a string representation of the given error.

    Arguments:
        error: The error for which a string is expected.
    
    Returns a string which represents the given error.
*/
const char* get_error_string(Error error);

/*
    Opens a capture session with the given width and height, using the camera
    located at the given device filepath. Then ensures that the chosen device
    supports the chosen resolution and data format.

    Once the above is checked, the buffers used for frame transfer are initialized.

    Parameters:
        session: The session struct used to maintain session state.
        device_filepath: The file path of the camera device (/dev/video<n>, where n = 0, 1, ...).
        width, height: The capture resolution.
    
    Returns:
        Error::OK on success, and a suitable error on failure:
            Error::OPEN: Failed to open the camera device file.
            Error::QUERY_CAPABILITIES: Failed to retrieve camera device capabilities.
            Error::NO_VIDEO_CAPTURE: The device does not support video capture.
            Error::NO_STREAMING: The device does not support video streaming.
            Error::QUERY_FORMAT: Failed to retrieve device pixel format and resolution.
            Error::SET_FORMAT: Failed to offer our desired format to the device.
            Error::UNSUPPORTED_FORMAT: The device does not support our format.
            Error::UNSUPPORTED_RESOLUTION: The device does not support the supplied resolution.
            Error::REQUEST_BUFFERS: Failed to request the ability to use our buffers.
            Error::LINK_BUFFERS: Failed to link our buffers to the device.
*/
Error open(CaptureSession* session, const char* device_filepath, size_t width, size_t height);

/*
    Starts the capture session. The device will begin to offer frames.

    Parameters:
        session: The capture session. This must first be initialized with `open`.
    
    Returns:
        Error::OK on success, and a suitable error on failure:
            Error::START_STREAM: Failed to start the capture stream.
*/
Error start(CaptureSession* session);

/*
    Grabs the next frame from the camera device.

    The frame buffer MUST be returned after it is processed by calling `return_buffer`.

    Parameters:
        session: The capture session. `start` must be called prior to this function.
    
    Return Parameters:
        out_frame: The frame data (on success).
        out_frame_size: The size of the captured frame (on success).

    Returns:
        Error::OK when a frame was read.
        Otherwise, a suitable error is returned:
            Error::SELECT: Failed while waiting to read from the device.
            Error::READ_FRAME: Failed to read raw frame data.
*/
Error grab_frame(CaptureSession* session, uint8_t** out_frame, size_t* out_frame_size);

/*
    Returns the frame buffer so that V4L can reuse it. MUST be called after
    `grab_frame`.

    Parameters:
        session: The capture session.
    
    Returns:
        Error::OK on success and a suitable error on failure:
            Error::PREPARE_BUFFER: Failed to prepare the buffer for another read.
*/
Error return_buffer(CaptureSession* session);

/*
    Stops capture and closes the device.

    Parameters:
        session: The capture session.
*/
void close(CaptureSession* session);

} // namespace camera

#endif
