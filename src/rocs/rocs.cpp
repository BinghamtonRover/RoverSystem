#include <linux/i2c-dev.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h> 
#include <unistd.h>
#include <fcntl.h>

#include "rocs.hpp"

namespace rocs {

int fd;
uint8_t last_slave = 0;

static Error select_slave(uint8_t slave_addr) {
    if (ioctl(fd, I2C_SLAVE, slave_addr) < 0) {
        return Error::SLAVE_SELECT;
    }

    return Error::OK;
}

Error init(const char* device) {
    fd = open(device, O_RDWR);
    if (fd < 0) return Error::OPEN;

    return Error::OK;
}

Error read_from_register(uint8_t slave_addr, uint8_t reg, uint8_t* out_value) {
    if (slave_addr != last_slave) {
        auto err = select_slave(slave_addr);
        if (err != Error::OK) return err;
    }

    if (write(fd, &reg, 1) != 1) {
        return Error::WRITE;
    }

    if (read(fd, out_value, 1) != 1) {
        return Error::READ;
    }

    return Error::OK;
}

Error write_to_register(uint8_t slave_addr, uint8_t reg, uint8_t value) {
    if (slave_addr != last_slave) {
        auto err = select_slave(slave_addr);
        if (err != Error::OK) return err;
    }

    uint8_t write_buffer[2] = { reg, value };

    if (write(fd, write_buffer, 2) != 2) {
        return Error::WRITE;
    }

    return Error::OK;
}

Error read_name(uint8_t slave_addr, char* out_name) {
    uint8_t len;

    auto err = read_from_register(slave_addr, 0, &len);
    if (err != Error::OK) return err;

    for (uint8_t i = 0; i < len; i++) {
        err = read_from_register(slave_addr, 0, reinterpret_cast<uint8_t*>(out_name));
        if (err != Error::OK) return err;

        out_name++;
    }

    *out_name = 0;

    return Error::OK;
}

#define X(s) [(int)Error::s] = #s
static const char* error_strings[] = {
    ROCS_ERROR_DEF(X)
};
#undef X

const char* get_error_string(Error e) {
    return error_strings[(int)e];
}

}
