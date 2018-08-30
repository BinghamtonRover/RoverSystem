This document describes the connection protocol between the base station and the rover.

# Connection States

Both the base station and the rover maintain independent __connection states__. The possible states are as follows:

* `CONNECTION_STATE_UNINITIALIZED` - No connection to the other side has been established since starting the program. Both the base station and the rover enter into this state initially.
* `CONNECTION_STATE_CONNECTED` - Both sides are connected (the critera for a connection are described below). This is the normal operation state.
* `CONNECTION_STATE_TROUBLED` - The other side has not sent a heartbeat packet within enough time to consider the state normal. Efforts are being made to contact the other side.
* `CONNECTION_STATE_DISCONNECTED` - The other side has stopped acknowledging the connection. Attempts are being made to re-establish communication.

# Constants

There are several constants that must be defined for the protocols below:

* `CONNECTION_ROVER_CHIRP_DELAY` - The time delay between consecutive heartbeat packets sent from the rover to the multicast discovery address ("chirps") for discovery.
* `CONNECTION_HEARTBEAT_NORMAL_DELAY` - The time delay between consecutive heartbeat packets sent from the base station to the rover ("BS->R"). This delay is used only when the rover is responding properly to heartbeat packets.
* `CONNECTION_HEARTBEAT_URGENT_DELAY` - The time delay between consecutive BS->R heartbeat packets when the base station is in `CONNECTION_STATE_TROUBLED` (protocol details are below).
* `CONNECTION_NORMAL_TIMEOUT` - The amount of time the base station will wait since the last received heartbeat packet before switching to `CONNECTION_STATE_TROUBLED`.
* `CONNECTION_URGENT_TIMEOUT` - The amount of time the base station will wait since the last received heartbeat packet while in `CONNECITON_STATE_TROUBLED`. Also the amount of time the rover will wait for incoming BS->R heartbeats before declaring `CONNECTION_STATE_DISCONNECTED`.
* `CONNECTION_DISCOVERY_ADDRESS` - The multicast IP address to use for discovery. It must be set to `233.252.66.85`.
* `CONNECTION_DISCOVERY_PORT` - The port to use for multicast discovery. It must be set to `44444`.

## Sane Defaults

The following defaults are recommended for the constants above:

* `CONNECTION_ROVER_CHIRP_DELAY` = `500 milliseconds`.
* `CONNECTION_HEARTBEAT_NORMAL_DELAY` = `1 second`.
* `CONNECTION_HEARTBEAT_URGENT_DELAY` = `250 milliseconds`.
* `CONNECTION_NORMAL_TIMEOUT` = `3 seconds`.
* `CONNECTION_URGENT_TIMEOUT` = `6 seconds`.

# Protocols

The base station and the rover have separate roles in the connection protocol. The base station, since it controls the rover, has the responsibility of initially finding the rover and sending heartbeat packets.

## Base Station

When the base station is initially started, it is in `CONNECTION_STATE_UNINITIALIZED`. The base station listens on `CONNECTION_DISCOVERY_ADDRESS` and port `CONNECTION_DISCOVERY_PORT`, and only accepts heartbeat packets. When a heartbeat `PING` is received, the base station records the address and port of the sender of the heartbeat packet (the rover). Then the base station enters `CONNECTION_STATE_CONNECTED` and begins sending heartbeat `PING` packets to the rover at interval `CONNECTION_HEARTBEAT_NORMAL_DELAY`.

If a `PONG` heartbeat is not received in `CONNECTION_NORMAL_TIMEOUT`, the rover enters `CONNECTION_STATE_TROUBLED` and begins sending heartbeat packets at interval `CONNECTION_HEARTBEAT_URGENT_DELAY`. If no `PONG` heartbeats are received in `CONNECTION_URGENT_TIMEOUT`, the base station enters `CONNECTION_STATE_DISCONNECTED` and attempts reconnection by listening on `CONNECTION_DISCOVERY_ADDRESS` and port `CONNECTION_DISCOVERY_PORT`. Once the rover is found through that process, the base station enters `CONNECTION_STATE_CONNECTED` and resumes polling with `CONNECTION_NORMAL_TIMEOUT`. On the other hand, if a `PONG` heartbeat is received in `CONNECTION_URGENT_TIMEOUT`, state is switched to `CONNECTION_STATE_CONNECTED` and the base station resumes polling with `CONNECTION_NORMAL_TIMEOUT`.

## Rover

When the rover is initially started, it is in `CONNECTION_STATE_UNINITIALIZED`. The rover begins listening at an arbitrary port and sends heartbeat `PING` packets  ("chirps") to `CONNECTION_DISCOVERY_ADDRESS` and port `CONNECTION_DISCOVERY_PORT` at interval `CONNECTION_ROVER_CHIRP_DELAY` until a heartbeat `PING` packet is received (on the listening port). At this point, the rover enters `CONNECITON_STATE_CONNECTED`. The rover responds to every base station heartbeat `PING` packet with an immediate `PONG` packet. If a heartbeat packet is not received within `CONNECTION_URGENT_TIMEOUT`, the rover enters `CONNECTION_STATE_DISCONNECTED` and resumes sending chirps to `CONNECTION_DISCOVERY_ADDRESS` and port `CONNECTION_DISCOVERY_PORT`. When a heartbeat `PING` is received, `CONNECTION_STATE_CONNECTED` is entered and the normal process of responding to heartbeats is resumed.
