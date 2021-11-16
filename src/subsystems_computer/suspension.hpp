#include <cstdint>

namespace suspension {

enum Side : uint8_t { LEFT, RIGHT };

enum Direction : uint8_t { FORWARD, BACKWARD };

#define SUSPENSION_ERRORS_DEF(X) \
    X(OK), X(DEVICE_NOT_RECOGNIZED), X(WRITE), X(READ)

#define X(error) error
enum class Error { SUSPENSION_ERRORS_DEF(X) };
#undef X

//I need to get this from Joon I think - JM
#define max_speed 30.0f
#define min_speed 3.0f

const char* get_error_string(Error e);

//int init();
Error init(uint8_t slave_addr);

//int update(Side side, Direction direction, uint8_t speed);
Error update(Side side, Direction direction, uint8_t speed);

Error stop(Side side);

} // namespace suspension
