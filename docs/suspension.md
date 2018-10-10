# Suspension API, Version 1

This document describes the rover's suspension API of the version indicated above.

The key words "MUST", "MUST NOT", "REQUIRED", "SHALL", "SHALL NOT", "SHOULD", "SHOULD NOT", "RECOMMENDED", "MAY", and "OPTIONAL" in this document are to be interpreted as described in RFC 2119.

## Purpose

The purpose of this API is to allow the rover main program to drive the wheel motors.

## API

The following API MUST be implemented as a C++14 library, using the C++14 ABI. Each call described below MUST be implemented as a function within the namespace `suspension`. Integer types described below MUST be implemented as those from `stdint.h` with the correct sign and bit count. Any other types described below MUST be defined or typedefed within the namespace `suspension`, with an exception made for pointer or array types. Enums MUST be defined with `enum class`.

### Types

#### Error

`Error` is an enum which describes the possible error states across all calls. It contains the following entries:
```
Error::OK = 0, // No error occured. This MUST be returned if a call completed successfully.
Error::INIT = 1, // An error occured during initialization.
Error::SET_LEFT = 2, // An error occured while attempting to set left wheel motor speed.
Error::SET_RIGHT = 3, // An error occured while attempting to set right wheel motor speed.
Error::QUIT = 4 // An error occured during cleanup.
```

Implementations MUST use `uint8_t` as the backing type of `Error`.

#### MVec

`MVec` is a vector which describes the current speed and direction of the left and right-side suspension movement. It MUST be implemented as a sign-magnitude 8-bit integer. Thus, the high bit indicates sign (`0` positive, `1` negative) and the remaining 7 bits indicate magnitude. A positive `MVec` indicates forward movement, and a negative `MVec` indicates backward movement. `MVec` magnitude indicates motor speed and allows up to 127 different non-zero speeds. Due to the sign-magnitude encoding, there are two possible representations of zero (`0x80` and `0x00`). Implementations MUST treat both as `0`.

Implementations MUST define `MVec` as a typedef of `uint8_t`.

### Constants

#### VERSION

A `constexpr uint8_t VERSION` MUST be defined. Implementations MUST set the value of `VERSION` to match the verison of the API which is implemented.

### Calls

#### Error init()

The `init` call takes no parameters and returns an error code as type `Error`. All initialization for the suspension API implementation SHOULD occur within this call. The possible error codes for this call are:
  * `Error::OK`: Initialization completed successfully.
  * `Error::INIT`: Initialization did not complete successfully.

Users of this API MUST NOT use any other calls prior to `init`.

#### Error quit()

The `quit` call takes no parameters and returns an error code as type `Error`. All cleanup for the suspension API implementation MUST occur whithin this call. All resources (memory, file, etc.) aquired during the execution of any other call MUST be released within this call. All motor movement MUST be ceased within this call. The possible error codes for this call are:
  * `Error::OK`: Cleanup completed successfully.
  * `Error::QUIT`: Cleanup did not complete successfully.

Users of this API MUST NOT use any calls other than `init` after using `quit`. Users SHALL NOT call `quit` unless `init` was called prior.

#### Error set_left(MVec)

The `set_left` call takes a single parameter of type `MVec` and returns an error code as type `Error`. This call sets the current speed and direction of the movement of the suspension's left side. The parameter is described as follows:
  * `MVec vec`: The new speed and direction of the movement of the suspension's left side. The implementation MUST maintain this speed and direction until another `set_left` call or a `quit`
  call.
The possible error codes for this call are:
  * `Error::OK`: The movement was updated successfully.
  * `Error::SET_LEFT`: The movement was not updated successfully.

#### Error set_right(MVec)

The `set_right` call is identical to `set_left`, except that `set_right` updates the movement of the suspension's right side.