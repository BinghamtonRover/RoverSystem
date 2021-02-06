#include "../rocs/rocs.hpp"

namespace rocs {

Error init(const char* device) {
    return Error::OK;
}

Error read_from_register(uint8_t slave_addr, uint8_t reg, uint8_t* out_value) {
    *out_value = 0x45;
    return Error::OK;
}

Error write_to_register(uint8_t slave_addr, uint8_t reg, uint8_t value) {
    return Error::OK;
}

Error read_name(uint8_t slave_addr, char* out_name) {
    *out_name = 0;
    return Error::OK;
}

#define X(s) #s
const char* error_strings[] = {
    ROCS_ERROR_DEF(X)
};
#undef X

const char* get_error_string(Error e) {
    return error_strings[static_cast<int>(e)];
}

}
