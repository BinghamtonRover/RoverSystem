# Suspension API, Version 1

This document describes the rover's suspension command API of the version indicated above.

The key words "MUST", "MUST NOT", "REQUIRED", "SHALL", "SHALL NOT", "SHOULD", "SHOULD NOT", "RECOMMENDED", "MAY", and "OPTIONAL" in this document are to be interpreted as described in RFC 2119.

## Purpose

This API defines the interface with the 2018-2019 IEEE Mars Rover Suspension system. 

This document is broken down into several components:

- Purpose
- General Functionality
	- Operations
	- Safety Mechanisms
  - Command Hierarchy
- Commands

## General Functionality
<!-- picture of setup -->
The API allows for manual control of the suspension system.

### Operations

We define the operations and their effects for both automated and manual control modes. 

#### Manual Control

The speed and direction of the left and right-side drive motors can be manaully selected.

### Safety Mechanisms

In order for the suspension system to continue movement, it requires constant reasurance. If a signal is not recieved from the computer within (5) seconds of the last, the drive motors will be immediately halted.

### Command Hierarchy

Command precidence is based on the time-of-execution, with the most-recent command taking precidence.

## Commands

The following commands MUST be sent over a serial connection. Each command is two bytes long. The first byte is the control byte, which specifies the command. The second byte is the value byte, which provides extra information relating to the command.

| Command Name | Command Byte | Value Byte | Description |
| - | - | - | - |
| STOP | 0x00 | Must be 0x00. | Instructs the suspension to cease all motor functions. |
| LMOV | 0x01 | Movement speed and direction of the left side motors, encoded as sign-magnitude. | Sets the movement speed and direction of the left side motors. Positive indicates forward movement, and negative indicates backward movement. Due to the sign-magnitude encoding, there are two zero values. Both MUST be interpreted as indicating zero motion. The encoding of the speed occupies the lower 7 bits of the value byte, so the range of the speed is [0, 127], where each speed value `s` equates to movement at `s/127` times maximum speed. |
| RMOV | 0x02 | Movement speed and direction of the right side motors, encoded as sign-magnitude. | Sets the movement speed and direction of the right side motors. Positive indicates forward movement, and negative indicates backward movement. Due to the sign-magnitude encoding, there are two zero values. Both MUST be interpreted as indicating zero motion. The encoding of the speed occupies the lower 7 bits of the value byte, so the range of the speed is [0, 127], where each speed value `s` equates to movement at `s/127` times maximum speed. |