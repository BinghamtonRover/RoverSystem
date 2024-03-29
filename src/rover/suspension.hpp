#include <cstdint>

namespace suspension {

enum Side : uint8_t { LEFT, RIGHT };

enum Direction : uint8_t { FORWARD, BACKWARD };

#define SUSPENSION_ERRORS_DEF(X) \
    X(OK), X(DEVICE_NOT_RECOGNIZED), X(WRITE), X(READ)

#define X(error) error
enum class Error { SUSPENSION_ERRORS_DEF(X) };
#undef X

const char* get_error_string(Error e);

Error init(uint8_t slave_addr);

Error update(Side side, Direction direction, uint8_t speed);

Error stop(Side side);

} // namespace suspension
