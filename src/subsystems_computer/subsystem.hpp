#ifndef SUBSYSTEM_HPP
#define SUBSYSTEM_HPP

#include "../network/network.hpp"

namespace subsystem {

enum class State : uint8_t {
    // No attempt has yet been made to connect to the subsystem.
    NOT_CONNECTED,

    // An initial attempt to connect to the subsystem is currently being made.
    CONNECTING,

    // The subsystem is connected and is operational.
    CONNECTED,

    // An error has occurred and an attempt to reconnect to the subsystem is being made.
    RECONNECTING,

    // Reconnecting to the subsystem failed, so the subsystem is considered dead.
    DISCONNECTED
};

struct Subsystem {
    uint8_t id;

    const char* name;
    State state;
};

// Assigns and returns an id and initializes state to NOT_CONNECTED;
uint8_t add(const char* name);

// Gets the subsystem with the given id. Returns nullptr if no such subsystem exists.
Subsystem* get(uint8_t id);

// Returns an array of all subsystems.
Subsystem* get_all(uint8_t* out_num);

// Sends subsystem update messages to the base station.
void send_update(network::Feed* rover_feed);

} // namespace subsystem

#endif
