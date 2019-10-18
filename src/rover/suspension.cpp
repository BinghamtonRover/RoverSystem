#include "suspension.hpp"

namespace suspension {

#define X(error) #error
const char* error_strings[] = { SUSPENSION_ERRORS_DEF(X) };
#undef X

const char* get_error_string(Error e) {
    return error_strings[(int) e];
}

} // namespace suspension
