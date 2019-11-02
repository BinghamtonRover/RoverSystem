#include "gps.hpp"
#include "subsystem.hpp"

namespace gps {

static uint8_t subsystem_id;

Error open() {
    subsystem_id = subsystem::add("GPS (dummy)");

    subsystem::get(subsystem_id)->state = subsystem::State::CONNECTED;

    return Error::OK;
}

bool has_fix() {
    return true;
}

Position get_current_position() {
    return { 42.087053, -75.967886 };
}

} // namespace gps
