#include "gps.hpp"
#include "subsystem.hpp"

namespace gps {

static uint8_t subsystem_id;

Error init(const char* device_id, util::Clock* global_clock) {
    subsystem_id = subsystem::add("GPS (dummy)");

    subsystem::get(subsystem_id)->state = subsystem::State::CONNECTED;

    return Error::OK;
}

network::LocationMessage::FixStatus get_fix() {
    return network::LocationMessage::FixStatus::FIXED;
}

Position get_position() {
    return { 42.087053, -75.967886 };
}

float get_heading() {
    return 100.0f;
}

} // namespace gps
