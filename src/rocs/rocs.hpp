#ifndef ROCS_HPP
#define ROCS_HPP

#include <stdint.h>

namespace rocs {

#define ROCS_ERROR_DEF(X) \
    X(OK), \
    \
    X(OPEN), \
    X(SLAVE_SELECT), \
    X(READ), \
    X(WRITE),

#define X(s) s
enum class Error {
    ROCS_ERROR_DEF(X)
};
#undef X

Error init(const char* device);

Error read_from_register(uint8_t slave_addr, uint8_t reg, uint8_t* out_value);
Error write_to_register(uint8_t slave_addr, uint8_t reg, uint8_t value);

Error read_name(uint8_t slave_addr, char* out_name);

const char* get_error_string(Error e);

}

#endif
