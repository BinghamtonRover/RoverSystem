#include "serial.hpp"

#include <stdio.h>

namespace serial {

constexpr int MAX_USB_SERIAL_DEVICES = 10;

const char* SERIAL_DEVICE_PATH_TEMPLATE = "/sys/bus/usb-serial/devices/ttyUSB%d/../../serial";

int get_device_number(char* serial_code) {
	// Strategy: Look at /sys/bus/usb-serial/devices/ttyUSB{n}/../../serial
	// for each USB serial device that exists.
	// Check to see if its serial is equal to the given serial.
	// If it is, return {n}.

	for (int i = 0; i < MAX_USB_SERIAL_DEVICES; i++) {
		char serial_file_name[sizeof(SERIAL_DEVICE_PATH_TEMPLATE) + 1];
		sprintf(serial_file_name, SERIAL_DEVICE_PATH_TEMPLATE, i);

		FILE* serial_file = fopen(serial_file_name, "r");
		if (!serial_file) continue;

		while (true) {
			int potential_c = fgetc(serial_file);
			if (potential_c == EOF) {
				if (*serial_code == 0) {
					fclose(serial_file);
					return i;
				} else {
					fclose(serial_file);
					break;
				}
			}

			if (*serial_code == 0) {
				fclose(serial_file);
				break;
			}

			char c = (char) potential_c;

			if (*serial_code != c) {
				fclose(serial_file);
				break;
			}

			serial_code++;
		}
	}

	return -1;
}

} // namespace serial
