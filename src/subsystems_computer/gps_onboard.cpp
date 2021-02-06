#include "gps.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <endian.h>
#include <math.h>
#include <stdint.h>
#include <termios.h>
#include <assert.h>
#include <sys/ioctl.h>

#include <thread>

namespace gps {

const uint32_t STABILIZATION_INTERVAL = 1000 * 5;

static util::Clock* global_clock;
static uint32_t last_fixed_millis = 0;
static uint32_t last_not_fixed_millis = 0;

static network::LocationMessage::FixStatus fix;

static Position last_position;
static float last_heading;

static const char DEVICE_SERIAL_BYID_PATH_TEMPLATE[] = "/dev/serial/by-id/%s";

static std::thread gps_thread;

const int LINE_BUFFER_SIZE = 1000;

static bool within_interval(uint32_t val, uint32_t interval) {
    return global_clock->get_millis() - val <= interval;
}

static void update_position(int sfd) {
    char line_buffer[LINE_BUFFER_SIZE];
    int buffer_size = 0;

    while (true) {

        buffer_size = 0;
        while (true) {
            bool have_fixed_recently = within_interval(last_fixed_millis, STABILIZATION_INTERVAL);
            bool have_not_fixed_recently = within_interval(last_not_fixed_millis, STABILIZATION_INTERVAL);

            if (have_fixed_recently && have_not_fixed_recently) {
                fix = network::LocationMessage::FixStatus::STABILIZING;
            } else if (have_fixed_recently && !have_not_fixed_recently) {
                fix = network::LocationMessage::FixStatus::FIXED;
            } else {
                fix = network::LocationMessage::FixStatus::NONE;
            }

            if (buffer_size >= LINE_BUFFER_SIZE - 1) {
                buffer_size = 0;
            }

            char c;

            if (read(sfd, &c, 1) != 1) continue;
            if (c == '\r') continue;

            line_buffer[buffer_size] = c;
            buffer_size++;

            if (line_buffer[buffer_size-1] == '\n') {
                line_buffer[buffer_size] = 0;
                break;
            }
        }

        if (strncmp(line_buffer, "$GPRMC", 6) == 0) {
            printf("GPRMC MESSAGE: %s\n", line_buffer);

            float lat;
            float lon;

            char temp_buffer[100];
            int temp_buffer_offset = 0;

            int offset = 0;
            int comma_count = 0;
            while (true) {
                if (offset >= buffer_size) break;

                if (line_buffer[offset] == ',') {
                    comma_count++;    
                    offset++;
                    continue;
                }

                if (comma_count == 2) {
                    // 'A' means fix, 'V' means no fix.
                    if (line_buffer[offset] == 'V') {
                        last_not_fixed_millis = global_clock->get_millis();
                        break;
                    }

                    last_fixed_millis = global_clock->get_millis();
                } else if (comma_count == 3) {
                    // Lat.
                    temp_buffer[temp_buffer_offset++] = line_buffer[offset];
                } else if (comma_count == 4) {
                    // N or S.

                    temp_buffer[temp_buffer_offset] = 0;

                    // First two chars are the degrees.
                    int deg = (temp_buffer[0] - '0') * 10 + (temp_buffer[1] - '0');

                    // The rest are the minutes.
                    float minutes = atof(temp_buffer + 2);

                    // One degree is 60 minutes.
                    lat = (float)deg + minutes/60.0f;

                    // S indicates -.
                    if (line_buffer[offset] == 'S') lat *= -1.0f;

                    temp_buffer_offset = 0;
                } else if (comma_count == 5) {
                    // Lon.
                    temp_buffer[temp_buffer_offset++] = line_buffer[offset];
                } else if (comma_count == 6) {
                    // E or W.

                    temp_buffer[temp_buffer_offset] = 0;

                    // First three chars are the degrees.
                    int deg = (temp_buffer[0] - '0') * 100 + (temp_buffer[1] - '0') * 10 + (temp_buffer[2] - '0');

                    // The rest are the minutes.
                    float minutes = atof(temp_buffer + 3);

                    // One degree is 60 minutes.
                    lon = (float)deg + minutes/60.0f;

                    // W indicates -.
                    if (line_buffer[offset] == 'W') lon *= -1.0f;

                    last_position.latitude = lat;
                    last_position.longitude = lon;

                    temp_buffer_offset = 0;
                } else if (comma_count == 8) {
                    // Heading.


                    temp_buffer[temp_buffer_offset++] = line_buffer[offset];
                } else if (comma_count == 9) {
                    // This is the date, but I dont care.
                    // This is just used to finish the heading.

                    temp_buffer[temp_buffer_offset] = 0;

                    last_heading = atof(temp_buffer);
                    break;
                }

                offset++;
            }
        }
    }
}

Error init(const char* device_id, util::Clock* clock) {
    global_clock = clock;

	char* device_path = (char*) malloc(sizeof(DEVICE_SERIAL_BYID_PATH_TEMPLATE) + strlen(device_id));
    sprintf(device_path, DEVICE_SERIAL_BYID_PATH_TEMPLATE, device_id);

    int sfd = open(device_path, O_RDWR | O_NOCTTY | O_SYNC);

    free(device_path);

	if (sfd == -1) {
		printf("Errno: %d\n", errno);
		return Error::OPEN;
	}

	struct termios tty;
	memset(&tty, 0, sizeof(tty));
	if (tcgetattr(sfd, &tty) != 0) {
		return Error::GET_ATTR;
	}

	cfsetospeed(&tty, B9600);
	cfsetispeed(&tty, B9600);

	if (tcsetattr(sfd, TCSANOW, &tty) != 0) {
		return Error::SET_ATTR;
	}

    gps_thread = std::thread(update_position, sfd);

    fix = network::LocationMessage::FixStatus::NONE;

	return Error::OK;
}

Position get_position() {
    return last_position;
}

float get_heading() {
    return last_heading;
}

network::LocationMessage::FixStatus get_fix() {
    return fix;
}

}
