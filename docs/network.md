# Rover Network Protocol Specification, Version 1

This document describes the network protocol used in all communication between base station and rover. First, the goals and requirements of the protocol are described. Then the protocol is defined in detail. Finally, the packet types used in the protocol are specified.

## Introduction

Due to the real-time nature of the Mars Society rover competition, this protocol has several guiding principles:
1. It must be fast. When in manual mode, the rover must respond to movement controls in real time. Since the base station and rover can be separated by up to a mile of distance, it is imperative that protocol overhead be as low as possible. This rules out the use of TCP, which uses buffering on both ends and can induce unpredictable delays; see [this article](https://gafferongames.com/post/udp_vs_tcp/) for details.
2. Not all packets must be sent reliably. With TCP, all data must receive acknowledgement; naturally, this induces delay. However, some packets in this protocol do not require receipt acknowledgement. These packets are typically sent at a fixed interval and contain state updates. In such cases, if a packet is dropped, no harm is done because another state update will arrive shortly.
3. Not all packets must be sent in-order. With TCP, the order of every bit of sent data is preserved. In this protocol, packets of different types which switch order may not cause a problem. In many cases, out-of-order packets can simply be dropped, particularly if such packets are sent at a regular interval. Additionally, in cases where data is stored and analyzed over time, out-of-order packets can still be accepted.

Due to the observations above, TCP is not a good choice for the base of this protocol. Instead, UDP is used, due to its low overhead. Reliable and in-order transmission are implemented per-packet on an as-needed basis.

## Protocol

All integer values in this protocol are big-endian encoded.

Each packet type in this protocol falls into one of four categories:
1. No ack and no order;
2. No ack and order;
3. Ack and no order;
4. Ack and order.

Ack packets require acknowledgement of packet receipt and order packets are discarded if they are out-of-order. The following describes examples for each category:

1. **No ack and no order**: Camera frame parts, sensor data;
2. **No ack and order**: Movement state changes (controller input in manual mode);
3. **Ack and no order**: Log messages;
4. **Ack and order**: Infrequent state changes, such as switching from manual to automatic mode.

### Packet Ordering

For each packet type which discards out-of-order packets, two *packet indices* are maintained: one for sent packets (`last_sent_idx`) and one for received packets (`last_received_idx`). Each starts at `0`. When sending a packet, `last_sent_idx` is incremented and the result is placed in the header of the outgoing packet. The packet index of each received packet (`incoming_idx`) is then compared to `last_received_idx`, If `incoming_idx` is not greater than the `last_received_idx`, the packet is discarded. Otherwise, `last_received_idx` is set to `incoming_idx`.

### Packet Acknowledgement

### Packet Header

All packet types share a common header. Its format is as follows:

| Field Name  | Field Type / Size |
| ----------- | ----------------- |
| Version     | u8                |
| Packet Type | u8                |

* `Version`: ...
* `Packet Type`: ...

If the packet type requires ordering, then the packet index follows:

| Field Name   | Field Type / Size |
| ------------ | ----------------- |
| Packet Index | u16               |

If the packet type requires acknowledgement, then

## Packet Specification