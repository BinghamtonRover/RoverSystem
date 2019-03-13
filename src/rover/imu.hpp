namespace imu {

enum class Error {
	OK,

	OPEN
};

struct Rotation {
	float pitch, yaw, roll;
};

Error start(const char* device_serial_id);
Rotation get_rotation();

}
