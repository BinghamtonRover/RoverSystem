#include "suspension.hpp"
#include <serial.hpp>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

namespace suspension {

const char DEVICE_PATH_TEMPLATE[] = "/dev/ttyUSB%d";

int serial_fd = -1;

Error init(const char* device_serial_code) {
	int device_number = serial::get_device_number(device_serial_code);
	if (device_number == -1) {
		return Error::DEVICE_NOT_FOUND;
	}

	char device_path[sizeof(DEVICE_PATH_TEMPLATE) + 1];
	sprintf(device_path, DEVICE_PATH_TEMPLATE, device_number);

	serial_fd = open(device_path, O_RDWR | O_NOCTTY | O_SYNC);
	if (serial_fd == -1) {
		return Error::OPEN_DEVICE;
	}

	return Error::OK;
}

Error update(Side side, Direction direction, uint8_t speed) {
	if (serial_fd == -1) return Error::INVALID_STATE;

	uint8_t side_enc = (uint8_t) side;	
	uint8_t direction_enc = (uint8_t) direction;

	if (dprintf(serial_fd, "%u\n%u\n%u\n", side_enc, direction_enc, speed_enc) < 0) {
		return Error::WRITE;
	}

	return Error::OK;
}

} // namespace suspension
