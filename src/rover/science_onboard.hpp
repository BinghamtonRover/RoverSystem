#include <cstdint>

namespace science {


#define SCIENCE_ERRORS_DEF(X) \
	X(OK), \
	X(INVALID_STATE), \
        X(OPEN_DEVICE), \
	X(GET_ATTR), \
	X(SET_ATTR), \
	X(WRITE)

#define X(error) error
enum class Error{
	SCIENCE_ERRORS_DEF(X)
};
#undef X

const char* get_error_string(Error e);

Error init(const char* device_serial_id);

Error update(uint16_t moisture, float pressure, float altitude, float temperature);
}