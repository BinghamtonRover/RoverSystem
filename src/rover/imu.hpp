namespace imu {

enum class Error {
	OK,

	OPEN
};

struct Rotation {
	float pitch, yaw, roll;
};

struct Velocity {
    float x, y, z;
};

Error start(const char* device_serial_id);
Rotation get_rotation();
Velocity get_velocity();

}
