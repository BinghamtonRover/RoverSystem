#include "../network/network.hpp"
#include "../util/util.hpp"

namespace gps {

struct Position {
    float latitude;
    float longitude;
};

enum class Error {
    OK,

    OPEN,
    GET_ATTR,
    SET_ATTR
};

Error init(const char* device_id, util::Clock* global_clock);

Position get_position();
float get_heading();

network::LocationMessage::FixStatus get_fix();

}
