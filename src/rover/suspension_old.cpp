#include "suspension.hpp"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
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

	return Error::OK;
}

Error update(Side side, Direction direction, uint8_t speed) {
	if (serial_fd == -1) return Error::INVALID_STATE;

	uint8_t side_enc = (uint8_t) side;	
	uint8_t direction_enc = (uint8_t) direction;

	if (dprintf(serial_fd, "%u\n%u\n%u\n", side_enc, direction_enc, speed) < 0) {
		return Error::WRITE;
	}

	char response_buffer[40];
	ssize_t bread = read(serial_fd, &response_buffer, 40);

	// printf("> Response: %.*s\n", bread, response_buffer);	

	return Error::OK;
}

} // namespace suspension
