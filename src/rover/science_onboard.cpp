#include "science_onboard.hpp"

namespace science{

#define X(error) #error
const char* error_strings[] = {
	SCIENCE_ERRORS_DEF(X)
};
#undef X

const char* get_error_string(Error e){
	return error_strings[(int) e];
}
//Not sure what to put here so i copied the same thing from suspension_old"
const char DEVICE_SERIAL_BYID_PATH_TEMPLATE[] = "/dev/serial/by-id/%s"

Error init(const char* device_serial_id){
	char* device_path = (char*) malloc(sizeof(DEVICE_SERIAL_BYID_PATH_TEMPLATE) + strlen(device_serial_id));
	sprintf(device_path, DEVICE_SERIAL_BYID_PATH_TEMPLATE, device_serial_id);

	serial_fd = open(device_path, O_RDWR | O_NOCTTY | O_SYNC);

	free(device_path);

	if(serial_fd == -1){
		printf("Errno: %d\n", errno);
		return Error::OPEN_DEVICE;
	}

	struct terminos tty;
	memset(&tty, 0, sizeof(tty));
	if (tcgetattr(serial_fd, &tty) != 0){
		return Error::GET_ATTR;
	}

	cfsetospeed(&tty, B9600);
	cfsetispeed(&tty, B9600);

	if(tcsetattr(serial_fd, TCSANOW, &tty) != 0){
		return Error::SET_ATTR;
	}
	return Error::OK;
}

Error update(uint16_t moisture, float pressure, float altitude, float temperature){
	if (serial_fd == -1) return Error::INVALID_STATE;

	uint16_t altitude_enc = (uint16_t) altitude;
	float pressure_enc = (float) pressure;
	float altitude_enc = (float) altitude;
	float temperature_enc = (float) temperature;

	if(dprintf(serial_fd, "%d %f %f %f\n", moisture_enc, pressure_enc, altitude_enc, temperature_enc) < 0){
		printf("error\n");
		return Error::WRITE;
	}

	return Error::OK;
}
}


