#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include <unistd.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

const int BP_BB_RESET_COUNT = 50;

enum class Error {
    OK,

    TIMEOUT,
    READ,
    WRITE
};

static Error serial_read(int fd, char* out_c) {
    fd_set set;
    FD_ZERO(&set);
    FD_SET(fd, &set);

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 50 * 1000;

    if (select(fd + 1, &set, NULL, NULL, &tv) < 0) {
        return Error::READ;
    }

    if (!FD_ISSET(fd, &set)) {
        return Error::TIMEOUT;
    }

    if (read(fd, out_c, 1) == -1) {
        return Error::READ;
    }

    return Error::OK;
}

static Error serial_write(int fd, char c) {
    if (write(fd, &c, 1) == -1) {
        return Error::WRITE;
    }

    return Error::OK;
}

static void bb_reset(int bp_fd) {
    for (int i = 0; i < BP_BB_RESET_COUNT; i++) {
        serial_write(bp_fd, 0x00);

        char resp[5];

        if (serial_read(bp_fd, resp) == Error::TIMEOUT) {
            continue;
        }

        for (int i = 1; i < 5; i++) {
            serial_read(bp_fd, resp + i);
        }

        if (strncmp("BBIO1", resp, 5) != 0) {
            fprintf(stderr, "[!] Received bad reset response (%.5s)!\n", resp);
            exit(1);
        }
    }
}

static char bb_wrap_read(int bp_fd) {
    char c;
    if (serial_read(bp_fd, &c) != Error::OK) {
        fprintf(stderr, "[!] Failed to read!\n");
        exit(1);
    }

    return c;
}

static void bb_wrap_write(int bp_fd, char c) {
    if (serial_write(bp_fd, c) != Error::OK) {
        fprintf(stderr, "[!] Failed to write!\n");
        exit(1);
    }
}

static void bb_check_response(int bp_fd, char expected) {
    char c = bb_wrap_read(bp_fd);
    if (c != expected) {
        fprintf(stderr, "[!] Bad BP response!\n");
        exit(1);
    }
}

static void bb_ignore_response(int bp_fd) {
    bb_wrap_read(bp_fd);
}

static void bb_enable_i2c(int bp_fd) {
    bb_wrap_write(bp_fd, 0x02);

    char resp[4];
    for (int i = 0; i < 4; i++) {
        resp[i] = bb_wrap_read(bp_fd);
    }

    if (strncmp("I2C1", resp, 4) != 0) {
        fprintf(stderr, "[!] Received bad I2C enable response %.4s!\n", resp);
        exit(1);
    }
}

static void bb_i2c_set_100khz(int bp_fd) {
    bb_wrap_write(bp_fd, 0x62);
    bb_check_response(bp_fd, 0x01);
}

static void bb_i2c_enable_pullups(int bp_fd) {
    // Enable power supply, enable pullups, disable other shit.
    bb_wrap_write(bp_fd, 0x4C);
    bb_check_response(bp_fd, 0x01);
}

static void bb_i2c_start(int bp_fd) {
    bb_wrap_write(bp_fd, 0x02); 
    bb_check_response(bp_fd, 0x01);
}

static void bb_i2c_write(int bp_fd, char val) {
    bb_wrap_write(bp_fd, 0x10);
    bb_check_response(bp_fd, 0x01);

    bb_wrap_write(bp_fd, val);
    bb_ignore_response(bp_fd);
}

static char bb_i2c_read(int bp_fd, bool read_another) {
    bb_wrap_write(bp_fd, 0x04);

    char c = bb_wrap_read(bp_fd);

    if (read_another) {
        bb_wrap_write(bp_fd, 0x06);
    } else {
        bb_wrap_write(bp_fd, 0x07);
    }
    bb_check_response(bp_fd, 0x01);

    return c;
}

static void bb_i2c_stop(int bp_fd) {
    bb_wrap_write(bp_fd, 0x03);
    bb_check_response(bp_fd, 0x01);
}

static uint8_t rocs_read(int bp_fd, uint8_t slave, uint8_t reg) {
    bb_i2c_start(bp_fd);

    // We need to write the address to the wire.
    // We need to write the register value, so indicate write.
    bb_i2c_write(bp_fd, (char)(slave << 1));
    bb_i2c_write(bp_fd, (char)reg);

    bb_i2c_start(bp_fd);

    // Now we need to read the value.
    bb_i2c_write(bp_fd, (char)((slave << 1) + 1));
    uint8_t v = (uint8_t) bb_i2c_read(bp_fd, false); 

    bb_i2c_stop(bp_fd);

    return v;
}

static void rocs_write(int bp_fd, uint8_t slave, uint8_t reg, uint8_t val) {
    bb_i2c_start(bp_fd);

    bb_i2c_write(bp_fd, (char)(slave << 1));
    bb_i2c_write(bp_fd, (char)reg);
    bb_i2c_write(bp_fd, (char)val);

    bb_i2c_stop(bp_fd);
}

int main(int argc, char** argv) {
    if (argc < 4) {
        fprintf(stderr, "[!] Args: <serial device path> <slave id> <register> [<write value>]\n");
        return 1;
    }

    int bp_fd = open(argv[1], O_RDWR | O_NOCTTY | O_SYNC);
    if (bp_fd == -1) {
        fprintf(stderr, "[!] Failed to open serial port!\n");
        return 1;
    }

    int slave_id_int = atoi(argv[2]);
    if (slave_id_int > 255 || slave_id_int < 0) {
        fprintf(stderr, "[!] Slave id out of bounds [0, 255]!\n");
        return 1;
    }

    int register_int = atoi(argv[3]);
    if (register_int > 255 || register_int < 0) {
        fprintf(stderr, "[!] Register out of bounds [0, 255]!\n");
        return 1;
    }

    uint8_t slave = (uint8_t)slave_id_int;
    uint8_t reg = (uint8_t)register_int;

    bool want_write = argc > 4;

    uint8_t write_value;
    if (want_write) {
        int write_value_int = atoi(argv[4]);
        if (write_value_int > 255 || write_value_int < 0) {
            fprintf(stderr, "[!] Value out of bounds [0, 255]!\n");
            return 1;
        }

        write_value = (uint8_t)write_value_int;
    }

	struct termios tty;
	memset(&tty, 0, sizeof(tty));
	if (tcgetattr(bp_fd, &tty) != 0) {
        fprintf(stderr, "[!] Failed to get serial info!\n");
        return 1;
	}

    tty.c_cflag = CS8 | CLOCAL | CREAD;
    tty.c_iflag = IGNPAR | BRKINT;
    tty.c_oflag = 0;
    tty.c_lflag = 0;

	cfsetospeed(&tty, B115200);
	cfsetispeed(&tty, B115200);

	if (tcsetattr(bp_fd, TCSANOW, &tty) != 0) {
        fprintf(stderr, "[!] Failed to set serial info!\n");
        return 1;
	}

#if 0
    // Hate serial.
    sleep(2);
    tcflush(bp_fd, TCIOFLUSH);
#endif

    bb_reset(bp_fd);     
    bb_enable_i2c(bp_fd);

    bb_i2c_set_100khz(bp_fd);
    bb_i2c_enable_pullups(bp_fd);

    if (reg == 0) {
        uint8_t len = rocs_read(bp_fd, slave, 0);

        for (uint8_t i = 0; i < len; i++) {
            printf("%c", rocs_read(bp_fd, 0x01, 0x00));
        }
        printf("\n");
    } else {
        if (want_write) {
            rocs_write(bp_fd, slave, reg, write_value);
        } else {
            printf("%u\n", rocs_read(bp_fd, slave, reg));
        }
    }

    return 0;
}
