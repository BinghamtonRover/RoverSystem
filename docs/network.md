# Rover Network Protocol Specification, Version 1

This document describes the network protocol used in all communication between base station and rover. First, the goals and requirements of the protocol are described. Then the protocol is defined in detail. Finally, the message types used in the protocol are specified.

## Index

* [Introduction](#introduction)
* [Protocol](#protocol)
    * [Message Indices](#message-indices)
    * [Message Ordering](#message-ordering)
    * [Message Acknowledgement](#message-acknowledgement)
    * [Packet Header](#packet-header)
    * [Message Header](#message-header)
* [Message Type Specification](#message-type-specification)
    * [0 - Heartbeat](#0---heartbeat)
    * [1 - Movement](#1---movement)

## Introduction

Due to the real-time nature of the Mars Society rover competition, this protocol has several guiding principles:
1. It must be fast. When in manual mode, the rover must respond to movement controls in real time. Since the base station and rover can be separated by up to a mile of distance, it is imperative that protocol overhead be as low as possible. This rules out the use of TCP, which uses buffering on both ends and can induce unpredictable delays; see [this article](https://gafferongames.com/post/udp_vs_tcp/) for details.
2. Not all packets must be sent reliably. With TCP, all data must receive acknowledgement; naturally, this induces delay. However, some packets in this protocol do not require receipt acknowledgement. These packets are typically sent at a fixed interval and contain state updates. In such cases, if a packet is dropped, no harm is done because another state update will arrive shortly.
3. Not all packets must be sent in-order. With TCP, the order of every bit of sent data is preserved. In this protocol, packets of different types which switch order may not cause a problem. In many cases, out-of-order packets can simply be dropped, particularly if such packets are sent at a regular interval. Additionally, in cases where data is stored and analyzed over time, out-of-order packets can still be accepted.

Due to the observations above, TCP is not a good choice for the base of this protocol. Instead, UDP is used, due to its low overhead. Reliable and in-order transmission are implemented per-packet on an as-needed basis.

## Protocol

All integer values in this protocol are big-endian encoded.

A single unit of communication is known as a _message_. Multiple messages can be transmitted per UDP packet.

Each message type in this protocol falls into one of four categories:
1. No ack and no order;
2. No ack and order;
3. Ack and no order;
4. Ack and order.

Ack messages require acknowledgement of message receipt and order messages are discarded if they are out-of-order. The following describes examples for each category:

1. **No ack and no order**: Camera frame parts, sensor data;
2. **No ack and order**: Movement state changes (controller input in manual mode);
3. **Ack and no order**: Log messages;
4. **Ack and order**: Infrequent state changes, such as switching from manual to automatic mode.

### Message Indices

Every message sent contains a unique index (disregarding rollover). The sender maintains the index of the last sent message (`last_sent_idx`) and the index of the last received message (`last_received_idx`). These indices are independent of each other. Each index starts as `0`. When sending a message, `last_sent_idx` is incremented and the result is placed in the header of the outgoing message.

Message indices are unsigned 16-bit integers. When they reach `65535`, they must roll over to `0`. All comparisons must be careful to take this into account: as an example, the message index `1` is "greater" (or "more recent") than a message index of `65534` but not greater than `2`. See [this](https://gafferongames.com/post/reliability_ordering_and_congestion_avoidance_over_udp#handling-sequence-number-wrap-around) for a method of properly comparing message indices.

### Message Ordering

For message types which are order-sensitive, the message index of each received message (`incoming_idx`) is then compared to `last_received_idx`. If `incoming_idx` is not greater than the `last_received_idx`, the packet is discarded. Otherwise, `last_received_idx` is set to `incoming_idx`.

### Message Acknowledgement

When a message is sent which requires acknowledgement, it must be kept in a buffer for easy lookup if it needs to be resent. If acknowledgement is received, then the message is removed.

The index of the last received ack message (`last_ack_idx`) must be maintaned. Additionally, an 32-bit bitfield (`ack_diff`) must also be maintained and sent within each UDP packet. Each bit in `ack_diff` represents an offset from `last_ack_idx`: bit 0 represents `last_ack_idx - 1`, bit 1 represents `last_ack_idx - 2`, and so forth. If bit 0 is set, then the message with index `last_ack_idx - 1` was received; if bit 1 is set, then the message with index `last_ack_idx - 2` was received, and so on. It is important to note that this offset must roll over: if `last_ack_idx == 0` then bit 0 represents a message with index `365535`. When a new ack message is received, `last_ack_idx` must be updated, and `ack_diff` must be shifted left by `new_last_ack_idx - old_last_ack_idx` bits. Note that this subtraction must be able to handle the case when `last_ack_idx` rolls over to 0.

The sender checks incoming ack information and remove any acked messages from its resend buffer.

### Packet Header

All UDP packets contain a common header. Its format is as follows:

| Field Name      | Field Type / Size |
| --------------- | ----------------- |
| Version         | u8                |
| Last Ack Index  | u16               |
| Ack Diff        | u32               |
| Num Messages    | u8                |

* `Version`: The version of this protocol.
* `Last Ack Index`: The index of the last ack message received by the sender.
* `Ack Diff`: The encoding of other recently acked packets.
* `Num Messages`: the number of messages contained in the packet.

This header is followed by one or more message headers.

### Message Header

Each message contains a header. Its format is as follows:

| Field Name    | Field Type / Size |
| ------------- | ----------------- |
| Index         | u16               |
| Type          | u8                |

* `Index`: The index of the given message.
* `Type`: The type of the given message.

This header is then followed by the message itself.

## Message Type Specification

This section describes the use, encoding, and procedure of each message type.

### 0 - Heartbeat

The heartbeat message simply alerts the receiving side that the sending side is awake. This message is only sent during periods of inactivity. It has no body.

Heartbeat messages are acked and are not order-sensitive.

### 1 - Movement

The movement message describes how the rover should be moving in manual mode. It contains two signed values, one for the left side motors and one for the right side. The magnitude of these values decribes the movement speed and the sign dictates forward or backward movement.

| Field Name | Field Type / Size |
| ---------- | ----------------- |
| Left       | s16               |
| Right      | s16               |

* `Left`: The speed (magnitude) and direction (sign) of the left side wheel motors.
* `Right`: The speed (magnitude) and direction (sign) of the right side wheel motors.

Movement messages are not acked and are order-sensitive.
