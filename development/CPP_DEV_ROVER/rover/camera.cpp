#include <iostream>

#include "camera.h"

// Converts a YCrCb color to RGB.
static void ycc_to_rgb(uint8_t y, uint8_t cb, uint8_t cr, uint8_t* r, uint8_t* g, uint8_t* b) {
    float fy = (float) y;
    float fcb = (float) cb;
    float fcr = (float) cr;

    float fr = fy + 1.402f * (fcr - 128.0f);
    float fg = fy - 0.344136f * (fcb - 128.0f) - 0.714136f * (fcr - 128.0f);
    float fb = fy + 1.772f  * (fcb - 128.0f);

    *r = (uint8_t) fr;
    *g = (uint8_t) fg;
    *b = (uint8_t) fb;
}

void Buffer::allocate(size_t _size) {
    size = _size;

    // Allocate enough data.
    data = new uint8_t[size];
}

Buffer::~Buffer() {
    // Free the memory.
    delete[] data;
}

CaptureSession::~CaptureSession() {
    // Close the connection to the camera.
    close(fd);
}


bool CaptureSession::open(std::string camera_path) {
    // Open the camera file. This returns -1 on error.
    fd = open(camera_path.c_str(), O_RDWR);
    if (fd == -1) {
        std::cerr << "[!] Failed to open camera file at " << camera_path << "!" << std::endl;
        return false;
    }

    return true;
}

bool CaptureSession::check_capabilities() {
    struct v4l2_capability cap;

    // Query for capability.
    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) != 0) {
        std::cerr << "[!] Failed to query capabilities!" << std::endl;
        return false;
    }

    std::cout << "> Camera name: \"" << (char*) cap.card << "\"" << std::endl;

    // Does the camera support video capture?
    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
        std::cerr << "[!] Camera does not support video capture!" << std::endl;
        return false;
    }

    // Does this camera support video streaming?
    if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
        std::cerr << "[!] Camera does not support video streaming!" << std::endl;
        return false;
    }

    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));

    // Query the video format.
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_G_FMT, &fmt) != 0) {
        std::cerr << "[!] Failed to query video format!" << std::endl;
        return false;
    }

    // Check the pixel format.
    if (fmt.fmt.pix.pixelformat != PIXEL_FORMAT) {
        std::cerr << "[!] Camera uses unsupported pixel format!" << std::endl;
        return false;
    }

    // Record the width and height of the camera.
    width = (size_t) fmt.fmt.pix.width;
    height = (size_t) fmt.fmt.pix.height;

    // Record the frame size (in bytes).
    image_size = (size_t) fmt.fmt.pix.sizeimage;

    std::cout << "> Image size: " << width << " x " << height << std::endl;

    return true;
}


bool CaptureSession::init_buffers() {
    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));

    req.count = NUM_BUFFERS; // Number of buffers to register.
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_USERPTR; // Specify the memory mode. We are using "user pointer" mode.

    // Request the ability to use our own buffers.
    if (ioctl(fd, VIDIOC_REQBUFS, &req) != 0) {
        std::cerr << "[!] Failed to request buffers!" << std::endl;
        return false;
    }

    // Allocate our buffers.
    for (size_t i = 0; i < NUM_BUFFERS; i++) {
        buffers[i].allocate(image_size);
    }

    // Link each buffer with the driver.
    for (size_t i = 0; i < NUM_BUFFERS; i++) {
            struct v4l2_buffer vbuf;
        memset(&vbuf, 0, sizeof(vbuf));

        vbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        vbuf.memory = V4L2_MEMORY_USERPTR;
        vbuf.index = i; // Which buffer is this?
        vbuf.m.userptr = (unsigned long) buffers[i].data; // Pointer to buffer's data.
        vbuf.length = buffers[i].size;

        // Link the buffer.
        if (ioctl(fd, VIDIOC_QBUF, &vbuf) != 0) {
            std::cerr << "[!] Failed to link buffer!" << std::endl;
            return false;
        }
    }

    return true;
}

bool CaptureSession::start_stream() {
    enum v4l2_buf_type buf_type;
    buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    // Start the video stream.
    if (ioctl(fd, VIDIOC_STREAMON, &buf_type) != 0) {
        std::cerr << "[!] Failed to start stream!" << std::endl;
        return false;
    }

    return true;
}

bool CaptureSession::grab_frame(cv::Mat& out_image) {
    // Set up object for the select call.
    fd_set fds;
    struct timeval tv;

    // Set up our file descriptors to watch (only fd).
    FD_ZERO(&fds);
    FD_SET(fd, &fds);

    // Set the timeout.
    tv.tv_sec = SELECT_TIMEOUT;
    tv.tv_usec = 0;

    // Wait for the next frame. See select(2) for more information.
    if (select(fd + 1, &fds, NULL, NULL, &tv) <= 0) {
        std::cerr << "[!] Failed to select!" << std::endl;
        return false;
    }

    // Set up the video4linux "buffer" (which points to our buffer).
    struct v4l2_buffer vbuf;
    memset(&vbuf, 0, sizeof(vbuf));
    
    vbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    vbuf.memory = V4L2_MEMORY_USERPTR;

    // Read the frame.
    if (ioctl(fd, VIDIOC_DQBUF, &vbuf) != 0) {
        // TODO: Handle EAGAIN.
        std::cerr << "[!] Failed to read frame!" << std::endl;
        return false;
    }

    // Get the pointer to our frame data.
    uint8_t* frame_data = (uint8_t*) vbuf.m.userptr;
    // Get the actual size of the frame data.
    size_t frame_size = vbuf.bytesused;

    // The YUYV format is stored in 4-byte chunks. Each 4-byte chunk defines
    // two pixels. The following link specifies the data format:
    // https://linuxtv.org/downloads/v4l-dvb-apis/uapi/v4l/pixfmt-yuyv.html.
    for (size_t i = 0; i < frame_size; i += 4) {
        uint8_t y0 = frame_data[i];     // Y component of first pixel.
        uint8_t cr = frame_data[i+1];   // Cr component of both pixels.
        uint8_t y1 = frame_data[i+2];   // Y component of second pixel.
        uint8_t cb = frame_data[i+3];   // Cb component of both pixels.

        uint8_t r, g, b;

        // First pixel conversion.
        ycc_to_rgb(y0, cb, cr, &r, &g, &b);

        // Set the appropriate pixel in the Mat.
        out_image.at<cv::Vec3b>(i/2)[0] = r;
        out_image.at<cv::Vec3b>(i/2)[0] = g;
        out_image.at<cv::Vec3b>(i/2)[0] = b;

        // Second pixel conversion.
        ycc_to_rgb(y1, cb, cr, &r, &g, &b);

        // Set the approprate pixel in the Mat.
        out_image.at<cv::Vec3b>((i/2)+1)[0] = r;
        out_image.at<cv::Vec3b>((i/2)+1)[1] = g;
        out_image.at<cv::Vec3b>((i/2)+1)[2] = b;
    }

    // Reset the buffer we just used.
    if (ioctl(fd, VIDIOC_QBUF, &vbuf) != 0) {
        std::cerr << "[!] Failed to prepare buffer for next read!" << std::endl;
        return false;
    }

    return true;
}