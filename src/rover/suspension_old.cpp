#include "suspension.hpp"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

namespace suspension {

int serial_fd = -1;

const char DEVICE_SERIAL_BYID_PATH_TEMPLATE[] = "/dev/serial/by-id/%s";

Error init(const char* device_serial_id) {
    char* device_path = (char*) malloc(sizeof(DEVICE_SERIAL_BYID_PATH_TEMPLATE) + strlen(device_serial_id));
    sprintf(device_path, DEVICE_SERIAL_BYID_PATH_TEMPLATE, device_serial_id);

    serial_fd = open(device_path, O_RDWR | O_NOCTTY | O_SYNC);

    free(device_path);

    if (serial_fd == -1) {
        printf("Errno: %d\n", errno);
        return Error::OPEN_DEVICE;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(serial_fd, &tty) != 0) {
        return Error::GET_ATTR;
    }

    cfsetospeed(&tty, B19200);
    cfsetispeed(&tty, B19200);

    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
        return Error::SET_ATTR;
    }

    return Error::OK;
}

Error update(Side side, Direction direction, uint8_t speed) {
    if (serial_fd == -1) return Error::INVALID_STATE;

    uint8_t side_enc = (uint8_t) side;
    uint8_t direction_enc = (uint8_t) direction;

    if (dprintf(serial_fd, "%u %u %u\n", side_enc, direction_enc, speed) < 0) {
        printf("error\n");
        return Error::WRITE;
    }

    /*
        char response_buffer[40];
        ssize_t bread = read(serial_fd, &response_buffer, 40);

        printf("> Response: %.*s\n", bread, response_buffer);
        */

    return Error::OK;
}

} // namespace suspension
