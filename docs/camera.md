# Camera Protocol, Version 2

Camera messages use the network protocol defined [in the network spec](network.md).

## Message Header

Each camera message has a common header:

| Field Name    | Field Size / Type |
| ------------- | ----------------- |
| Stream Index  | u8                |
| Frame Index   | u16               |
| Section Index | u8                |
| Section Count | u8                |
| Size          | u16               |

* `Stream Index` - An identifier for the camera stream which contains this message.
* `Frame Index` - A running index assigned to each frame.
* `Section Index` - The section of the frame being sent.
* `Section Count` - The total number of sections for this frame.
* `Size` - The size, in bytes, of the raw data of this section.

The raw frame data follows the header.

## Constants

There are several constants which need to be defined on the sending and/or receiving sides. They are as follows:
	
1. `CAMERA_MESSAGE_FRAME_DATA_MAX_SIZE = 65000`: The maximum size, in bytes, of the frame data in a single camera message. This should be small enough that a single camera message can fit within a UDP packet sent by the network protocol.
2. `CAMERA_FRAME_BUFFER_SIZE = 6220800`: The size, in bytes, of any buffer that will hold a frame. In other words, the maximum size of any single frame.
3. `CAMERA_FRAME_BUFFER_COUNT`:	The number of frames to collect on the receiving side before pushing frames to a view. The receiving side will have `CAMERA_FRAME_BUFFER_COUNT` buffers, each of size `CAMERA_FRAME_BUFFER_SIZE`, to hold frames. This value can be adjusted, as it only pertains to the base station. A higher `CAMERA_FRAME_BUFFER_COUNT` will result in a higher delay, but less skipped frames.

## Regarding Camera Streams

The following protocols and examples assume a single stream, and thus elide the `Stream Index` field. To handle multiple streams, the receiving side must allocate several collections of buffers, with one such collection associated with each stream.

## Sending Camera Messages

The sending side protocol:

1. Capture frame `F` from the camera.
2. Encode frame `F` into a JPEG image.
3. Let `F_s` be the size, in bytes, of `F`. Then construct `(F_s / CAMERA_MESSAGE_FRAME_DATA_MAX_SIZE) + 1` camera messages and
    distribute the contents of `F` across them, setting section index starting at `0` and ending at 
    `F_s / CAMERA_MESSAGE_FRAME_DATA_MAX_SIZE`. Set the section count of each message to `(F_s / CAMERA_MESSAGE_FRAME_DATA_MAX_SIZE) + 1`.
    Set the size field of each camera message to `CAMERA_MESSAGE_FRAME_DATA_MAX_SIZE`, except for the last message, which must have size
    `F_s % CAMERA_MESSAGE_FRAME_DATA_MAX_SIZE`.
4. Assign the next available frame index to all messages carrying `F`. The messages carrying `F` must all have the same
    frame index value.
5. Send the messages.

Here is an example. Say the latest frame from the camera, when encoded as a JPEG image, has a size of 141330 bytes.
Assume a `CAMERA_MESSAGE_FRAME_DATA_MAX_SIZE` of 40000 bytes. Then 4 messages will be created. Assume that the next frame index is 1234. Then the messages for this frame should appear as the following:

	| Frame Index: 1234   |   | Frame Index: 1234   |   | Frame Index: 1234   |   | Frame Index: 1234   |
	| Section Index: 0    |   | Section Index: 1    |   | Section Index: 2    |   | Section Index: 3    |
	| Section Count: 4    |	  | Section Count: 4    |   | Section Count: 4    |   | Section Count: 4    |
	| Size: 40000         |   | Size: 40000         |   | Size: 40000         |   | Size: 21330         |
	| <Frame data>        |   | <Frame data>        |   | <Frame data>        |   | <Frame data>        |

## Receiving Camera Packets

The receiving side protocol:

1. Before receiving any messages, set up `CAMERA_FRAME_BUFFER_COUNT` buffers, each of `CAMERA_FRAME_BUFFER_SIZE`. These buffers must be 'smart', in that each buffer records the frame index of the frame it represents. Each buffer must also record the number of sections it has left to receive for the frame it represents. The next "available" buffer must be recorded. When a buffer is full, the next buffer record must be updated to the next available buffer. If the next buffer has not yet been pushed to screen, then that frame is overwritten and dropped. Once the record reaches the last buffer, the record is reset to the first buffer.
2. A camera message is received. If one of the buffers contains the frame index of the message, then the message is providing another section of the frame within that buffer. Let `P_s` be the section index of the message. Then the frame data of the message is copied into the proper buffer at the offset `CAMERA_MESSAGE_FRAME_DATA_MAX_SIZE * P_s`. That buffer's remaining count is decremented. If the remaining count is equal to zero, the buffer is full and the frame has been fully received. If none of the buffers contain the frame index of the message, then a new frame is being received. The next available buffer must be reserved for this frame, and the next buffer record must be updated.

Here is an example. Assume that `CAMERA_FRAME_BUFFER_COUNT` is 2. Thus we have two buffers:

	| BUFFER 0       |   | BUFFER 1       |
	| Frame Index: 0 |   | Frame Index: 0 |
	| Remaining: 0   |   | Remaining: 0   |
	| <Empty>        |   | <Empty>        |

and a next buffer of 0.

A camera message is received. It has frame index of 21. Since neither buffer has a frame index of 21, this is a new frame. The message has a section index of 0 and a section count of 4. The current buffer (BUFFER 0) is updated and the frame data is copied. The next buffer is set to 1.

	| BUFFER 0         |   | BUFFER 1       |
	| Frame Index: 21  |   | Frame Index: 0 |
	| Remaining: 3     |   | Remaining: 0   |
	| <Section 0 Data> |   | <Empty>        |
	| <Empty>          |

Another camera message is received. It has frame index 21, which is assigned to BUFFER 0. Thus this is not a new frame. The message has a section index of 2 and a section count of 4. Notice this means that section 1 and section 2 are arriving out-of-order, but that is okay. BUFFER 0 is updated.

	| BUFFER 0         |   | BUFFER 1       |
	| Frame Index: 21  |   | Frame Index: 0 |
	| Remaining: 2     |   | Remaining: 0   |
	| <Section 0 Data> |   | <Empty>        |
	| <Empty>          |
	| <Section 2 Data> |
	| <Empty>          |

Another camera message is received. It has frame index 21, which is assigned to BUFFER 0. The message has a section index of 1 and a section count of 4. Buffer 0 is updated.

	| BUFFER 0         |   | BUFFER 1       |
	| Frame Index: 21  |   | Frame Index: 0 |
	| Remaining: 1     |   | Remaining: 0   |
	| <Section 0 Data> |   | <Empty>        |
	| <Section 1 Data> |
	| <Section 2 Data> |
	| <Empty>          |

Another camera message is received. It has frame index 26, which is not assigned to a buffer. Thus the next buffer is updated to 0, and BUFFER 1 is updated accordingly. The message has a section index of 0 and a section count of 3.

	| BUFFER 0         |   | BUFFER 1         |
	| Frame Index: 21  |   | Frame Index: 26  |
	| Remaining: 1     |   | Remaining: 2     |
	| <Section 0 Data> |   | <Section 0 Data> |
	| <Section 1 Data> |   | <Empty>          |
	| <Section 2 Data> |
	| <Empty>          |

Another camera message is received. It has frame index 21, which is assigned to BUFFER 0. The message has a section index of 3 and a section count of 4. Now BUFFER 0 is full, since its section count equals the message's section count. BUFFER 0 is pushed to screen.

	| BUFFER 0         |   | BUFFER 1         |
	| Frame Index: 21  |   | Frame Index: 26  |
	| Remaining: 4     |   | Remaining: 1     |
	| <Section 0 Data> |   | <Section 0 Data> |
	| <Section 1 Data> |   | <Empty>          |
	| <Section 2 Data> |
	| <Section 3 Data> |

Another camera message is received. It has frame index 28, which is not assigned to a buffer. Since the next buffer is 0 and BUFFER 0 has a non-zero frame index (meaning that it has at least a partial frame), we check its remaining count. Since that is 0, the buffer is full and thus not dropped. BUFFER 0 is then reset with the information from the new message, and the next buffer is updated to 1. The message has a section index of 1 and a section count of 2.

	| BUFFER 0         |   | BUFFER 1         |
	| Frame Index: 28  |   | Frame Index: 26  |
	| Remaining: 1     |   | Remaining: 1     |
	| <Empty>          |   | <Section 0 Data> |
	| <Section 1 Data> |   | <Empty>          |

Yet another camera message is received. It has frame index 29, which is not assigned to a buffer. Since the next buffer is 1 and BUFFER 1 has a non-zero frame index, we check its remaining count. Since that is non-zero, we have not received all the messages of frame 26, so the frame is dropped. Thus the next buffer is BUFFER 0 and BUFFER 1 is updated. The new message has a section index of 0 and a section count of 2.

	| BUFFER 0         |   | BUFFER 1         |
	| Frame Index: 28  |   | Frame Index: 29  |
	| Remaining: 1     |   | Remaining: 1     |
	| <Empty>          |   | <Section 0 Data> |
	| <Section 1 Data> |   | <Empty>          |

Yet another camera message is received. It has frame index 26, which is not assigned to a buffer. This was the frame that was just discarded! But we don't know that, so the next buffer is 0 and we discard BUFFER 0 since it has a non-zero remaining count. The last frame has a section index of 1 and a section count of 2.

	| BUFFER 0         |   | BUFFER 1         |
	| Frame Index: 26  |   | Frame Index: 29  |
	| Remaining: 1     |   | Remaining: 1     |
	| <Section 0 Data> |   | <Section 0 Data> |
	| <Empty>          |   | <Empty>          |

Of course, the next camera message to arrive has frame index of 28, which we just discarded. Since the next buffer is 1, BUFFER 1 is discarded and filled with the new information. The new message has a section index of 0 and a section count of 2.

	| BUFFER 0         |   | BUFFER 1         |
	| Frame Index: 28  |   | Frame Index: 29  |
	| Remaining: 1     |   | Remaining: 1     |
	| <Section 0 Data> |   | <Section 0 Data> |
	| <Empty>          |   | <Empty>          |

The next camera message has a frame index of 29, a section index of 1, and a section count of 2. BUFFER 1 contains this message, so BUFFER 1 is filled. The next buffer is updated to 0. BUFFER 1 is then pushed to screen.

	| BUFFER 0         |   | BUFFER 1         |
	| Frame Index: 28  |   | Frame Index: 29  |
	| Remaining: 1     |   | Remaining: 0     |
	| <Section 0 Data> |   | <Section 0 Data> |
	| <Empty>          |   | <Section 1 Data> |

The next camera message has a frame index of 30, a section index of 0, and a section count of 3. BUFFER 0 is the next buffer, so it is discarded (since it has a non-zero remaining count) and the next message's information is filled.

	| BUFFER 0         |   | BUFFER 1         |
	| Frame Index: 30  |   | Frame Index: 29  |
	| Remaining: 2     |   | Remaining: 0     |
	| <Section 0 Data> |   | <Section 0 Data> |
	| <Empty>          |   | <Section 1 Data> |

This process is continued indefinitely.

It is important to note that in the above example, two frames were rendered and two frames were lost (even though none of their messages were lost). This is mostly due to the tiny `CAMERA_FRAME_BUFFER_COUNT` value of 2. With a higher value (around 5), much more message reordering is tolerable. If a message is dropped, the frame will never be completed, which will result in more efficient buffer usage. Thus a higher `CAMERA_FRAME_BUFFER_COUNT` will avoid dropping most, if not all, frames with out-of-order messages.

A reference C++ implementation of the receiving side is available in /BinghamtonRover/legacy/scratch/CPP/camerafeed/receiver.
