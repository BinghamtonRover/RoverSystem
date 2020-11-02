#include <cstddef>
#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <unistd.h>
#include <stdio.h>

#include "camera.hpp"

namespace camera {

// List of string names for the error values.
#define X(name) #name
const char* error_names[] = { ERROR_DEF(X) };
#undef X

const char* get_error_string(Error error) {
    return error_names[(int) error];
}

Error open(CaptureSession* session, const char* device_filepath, size_t width, size_t height, uint8_t dev_id, util::Clock* clock, int framerate) {
    // Set the width and height.
    session->width = width;
    session->height = height;
    session->dev_video_id = dev_id;

    // Call Linux open on the device file.
    session->fd = ::open(device_filepath, O_RDWR);
    // On failure, open returns -1.
    if (session->fd == -1) {
        return Error::OPEN;
    }

    // Allocate a capability struct since we will need it several times.
    struct v4l2_capability cap;

    // The ioctls below return non-zero values on error.

    // Query for capability.
    if (ioctl(session->fd, VIDIOC_QUERYCAP, &cap) != 0) {
        return Error::QUERY_CAPABILITIES;
    }

    // Keep the camera's name just in case we want this later.
    strcpy(session->name, (char*) cap.card);

    // Also keep the camera's hardware location.
    strcpy(session->hardware_location, (char*) cap.bus_info);

    // Does the camera support video capture?
    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
        return Error::NO_VIDEOCAPTURE;
    }

    // Does this camera support video streaming?
    if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
        return Error::NO_STREAMING;
    }

    // Allocate a format struct for the following calls.
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));

    // Instruct V4L2 to ask for video format.
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    // Let V4L2 fill it in based on what is currently set.
    // This is so that we only need to set relevant values.
    if (ioctl(session->fd, VIDIOC_G_FMT, &fmt) != 0) {
        return Error::QUERY_FORMAT;
    }

    // Then set the pixel format we want.
    // Also set our desired resolution.
    fmt.fmt.pix.pixelformat = PIXEL_FORMAT;
    fmt.fmt.pix.width = session->width;
    fmt.fmt.pix.height = session->height;

    // Return the format with our changes.
    if (ioctl(session->fd, VIDIOC_S_FMT, &fmt) != 0) {
        return Error::SET_FORMAT;
    }

    // Check the returned pixel format.
    // If the camera does not support the pixel format we desire,
    // it will return whatever it can use.
    if (fmt.fmt.pix.pixelformat != PIXEL_FORMAT) {
        return Error::UNSUPPORTED_FORMAT;
    }

    // Check the returned dimensions.
    if (fmt.fmt.pix.width != session->width || fmt.fmt.pix.height != session->height) {
        return Error::UNSUPPORTED_RESOLUTION;
    }

    // Record the frame size (in bytes).
    session->image_size = (size_t) fmt.fmt.pix.sizeimage;

    //
    // Buffer Initialization.
    //

    // Allocate and clear a requestbuffers struct, since we need it later.
    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));

    // Number of buffers to register.
    req.count = NUM_BUFFERS;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    // Specify the memory mode. We are using "user pointer" mode.
    // We will supply pointers to buffers we allocate.
    req.memory = V4L2_MEMORY_USERPTR;

    // Request the ability to use our own buffers.
    if (ioctl(session->fd, VIDIOC_REQBUFS, &req) != 0) {
        return Error::REQUEST_BUFFERS;
    }

    // Allocate our buffers.
    for (uint32_t i = 0; i < NUM_BUFFERS; i++) {
        session->buffers[i].size = session->image_size;
        session->buffers[i].data = new uint8_t[session->image_size];
    }

    // Link each buffer with the driver.
    for (uint32_t i = 0; i < NUM_BUFFERS; i++) {
        // Allocate a V4L2 "buffer".
        // This really isn't a buffer, but a struct that contains information
        // about the buffers we will provide.
        struct v4l2_buffer vbuf;
        memset(&vbuf, 0, sizeof(vbuf));

        vbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        // USERPTR mode: provide our own pointers.
        vbuf.memory = V4L2_MEMORY_USERPTR;
        // Which buffer is this?
        vbuf.index = i;
        // Pointer to buffer's data.
        vbuf.m.userptr = (unsigned long) session->buffers[i].data;
        // How big is the buffer?
        vbuf.length = session->buffers[i].size;

        // Link the buffer.
        if (ioctl(session->fd, VIDIOC_QBUF, &vbuf) != 0) {
            return Error::LINK_BUFFERS;
        }
    }

    // Initialize the frame buffer.
    session->frame_buffer = new uint8_t[session->image_size];

    // Start the timer.
    util::Timer::init(&(session->timer), framerate, clock);

    return Error::OK;
}

Error start(CaptureSession* session) {
    // Set an enum value to let V4L2 know that we want to start video capture.
    enum v4l2_buf_type buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    // Start the video stream.
    if (ioctl(session->fd, VIDIOC_STREAMON, &buf_type) != 0) {
        return Error::START_STREAM;
    }

    return Error::OK;
}

Error grab_frame(CaptureSession* session, uint8_t** out_frame, size_t* out_frame_size) {
    // Set up structs for the select call.
    fd_set fds;
    struct timeval tv;

    // Set up our file descriptors to watch (only fd).
    FD_ZERO(&fds);
    FD_SET(session->fd, &fds);

    // Set the timeout.
    
    tv.tv_sec = 0;
    //tv.tv_usec = SELECT_TIMEOUT * 1000;
    tv.tv_usec = 0;

    // Wait for the next frame. See man select(2) for more information.
    if (select(session->fd + 1, &fds, NULL, NULL, &tv) <= 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
            return Error::AGAIN;

        return Error::SELECT;
    }

    // Set up the video4linux "buffer" (which points to our buffer).
    struct v4l2_buffer vbuf;
    memset(&vbuf, 0, sizeof(vbuf));

    vbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    vbuf.memory = V4L2_MEMORY_USERPTR;

    // Read the frame.
    if (ioctl(session->fd, VIDIOC_DQBUF, &vbuf) != 0) {
        return Error::READ_FRAME;
    }

    // Get the pointer to our frame data.
    *out_frame = (uint8_t*) vbuf.m.userptr;

    // Get the actual size of the frame data.
    *out_frame_size = vbuf.bytesused;

    // Set the buffer we used!
    session->last_capture_buffer = vbuf;

    // Now that we actually got a frame, only return it if we actually need it.
    if (!session->timer.ready()) {
        return_buffer(session);
        return Error::AGAIN;
    }

    return Error::OK;
}

Error return_buffer(CaptureSession* session) {
    // Reset the buffer we just used.
    if (ioctl(session->fd, VIDIOC_QBUF, &session->last_capture_buffer) != 0) {
        return Error::PREPARE_BUFFER;
    }

    return Error::OK;
}

void close(CaptureSession* session) {
    ::close(session->fd);

    for (int i = 0; i < NUM_BUFFERS; i++) {
        delete[] session->buffers[i].data;
    }
}

} // namespace camera
