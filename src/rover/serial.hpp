namespace serial {

// Returns the serial USB "device number" for the USB device with the given serial code.
// The device number {n} is the one found at the end of a USB serial device's /dev file:
// /dev/ttyUSB{n}.
// Returns -1 if no device is found with that serial code.
int get_device_number(char* serial_code);

} // namespace serial
