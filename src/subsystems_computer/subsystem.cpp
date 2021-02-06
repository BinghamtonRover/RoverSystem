#include "subsystem.hpp"

#include <assert.h>
#include <string.h>

namespace subsystem {

// This can't be more than 255!
static const int MAX_SUBSYSTEMS = 50;

static Subsystem subsystems[MAX_SUBSYSTEMS];
static uint8_t num_subsystems = 0;

uint8_t add(const char* name) {
    assert(num_subsystems < MAX_SUBSYSTEMS);
    
    uint8_t id = num_subsystems;

    subsystems[num_subsystems].id = id;
    subsystems[num_subsystems].name = strdup(name);
    subsystems[num_subsystems].state = State::NOT_CONNECTED;

    num_subsystems++;

    return id;
}

Subsystem* get(uint8_t id) {
    if (id >= num_subsystems) {
        return nullptr;
    }

    return subsystems + id;
}

Subsystem* get_all(uint8_t* out_num) {
    *out_num = num_subsystems;

    return subsystems;
}

void send_update(network::Feed* rover_feed) {
    // Space available for the list of subsystems.
    static uint8_t status_send_buffer[network::MAX_MESSAGE_SIZE - sizeof(network::SubsystemMessage)];
    const auto MAX_SUBSYSTEMS_PER_MESSAGE = sizeof(status_send_buffer) / sizeof(network::SubsystemMessage::Subsystem);

    network::SubsystemMessage subsystem_message;
    subsystem_message.subsystems = reinterpret_cast<network::SubsystemMessage::Subsystem*>(status_send_buffer);

    uint8_t num_subsystems;
    auto subsystems = subsystem::get_all(&num_subsystems);

    // TODO: Make this split into multiple packets!
    if (num_subsystems > MAX_SUBSYSTEMS_PER_MESSAGE) num_subsystems = MAX_SUBSYSTEMS_PER_MESSAGE;

    subsystem_message.num = num_subsystems;

    for (uint8_t i = 0; i < num_subsystems; i++) {
        auto subsystem = subsystem_message.subsystems + i;

        subsystem->id = subsystems[i].id;
        subsystem->set_name(subsystems[i].name);
        subsystem->state = static_cast<uint8_t>(subsystems[i].state);
    }

    network::publish(rover_feed, &subsystem_message);
}

} // namespace subsystem
