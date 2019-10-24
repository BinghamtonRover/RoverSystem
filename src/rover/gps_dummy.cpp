#include "gps.hpp"

namespace gps {

bool has_fix() {
    return true;
}

Position get_current_position() {
    return { 42.087053, -75.967886 };
}

} // namespace gps
