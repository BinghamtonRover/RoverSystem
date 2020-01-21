#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>

#include "../../src/rocs/rocs.hpp"

int main(int argc, char** argv) {
    if (argc < 3) {
        printf("[!] Args: <slave> <reg> [<value>]\n");
        return 1;
    }

    auto err = rocs::init("/dev/i2c-0");
    if (err != rocs::Error::OK) {
        fprintf(stderr, "[!] Failed to init ROCS: %s\n", rocs::get_error_string(err));
        return 1;
    }

    uint8_t slave_addr = (uint8_t) atoi(argv[1]);
    uint8_t reg = (uint8_t) atoi(argv[2]);

    if (argc > 3) {
        uint8_t val = (uint8_t) atoi(argv[3]);
        err = rocs::write_to_register(slave_addr, reg, val);
        if (err != rocs::Error::OK) {
            fprintf(
                stderr, 
                "[!] Failed to write to %u.%u: %s\n",
                slave_addr,
                reg,
                rocs::get_error_string(err));
            return 1;
        }
    } else {
        if (reg == 0) {
            static char name_buffer[257];

            err = rocs::read_name(slave_addr, name_buffer);
            if (err != rocs::Error::OK) {
                fprintf(stderr, "[!] Failed to get name of %u: %s\n", slave_addr, rocs::get_error_string(err));
                return 1;
            }

            printf("Name: %s\n", name_buffer);
        } else {
            uint8_t value;
            err = rocs::read_from_register(slave_addr, reg, &value);
            if (err != rocs::Error::OK) {
                fprintf(
                    stderr, 
                    "[!] Failed to read from %u.%u: %s\n",
                    slave_addr,
                    reg,
                    rocs::get_error_string(err));
            }

            printf("%u\n", value);
        }
    }


    return 0;
}
