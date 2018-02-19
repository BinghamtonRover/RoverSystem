package com.github.zeldazach.binghamtonrover.networking;

import com.github.zeldazach.binghamtonrover.gui.DisplayApplication;
import javafx.application.Platform;
import javafx.scene.image.Image;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;


class FrameBuffer {
    // The maximum size of a single frame.
    private static final int FRAME_MAX_SIZE = 40000000;

    int timestamp, remaining_sections, buffer_size;
    byte[] internal_buffer = new byte[FRAME_MAX_SIZE];

    FrameBuffer() {
        timestamp = remaining_sections = buffer_size = 0;
    }

    void push() {
        byte[] frame_buffer = new byte[buffer_size];

        // Copy only the frame bytes out of our buffer.
        System.arraycopy(internal_buffer, 0, frame_buffer, 0, buffer_size);

        Image image = new Image(new ByteArrayInputStream(frame_buffer));
        Platform.runLater(() -> {
            if (DisplayApplication.INSTANCE != null) {
                DisplayApplication.INSTANCE.getCameraImageView().setImage(image);
            }
        });
    }
}

class FrameBufferContainer {
    private int total_frames_received = 0;
    private int total_frames_dropped = 0;
    private int next_buffer;
    private FrameBuffer[] buffers;

    private int maxSize;

    FrameBufferContainer(int maxSize) {
        this.maxSize = maxSize;
        next_buffer = 0;
        buffers = new FrameBuffer[maxSize];

        for (int i = 0; i < maxSize; i++) {
            buffers[i] = new FrameBuffer();
        }
    }

    void update_buffer(PacketCamera packet) {
        int timestamp = packet.getHeader().getTimestamp();
        int section_count = packet.getSectionCount();
        int section_id = packet.getSectionIndex();
        byte[] frame_data = packet.getSectionData();
        int frame_data_size = packet.getSectionSize();

        // Search for a buffer that already has that timestamp.
        int found_buffer_idx = -1;
        for (int i = 0; i < maxSize; i++) {
            if (buffers[i].timestamp == timestamp) {
                // Found it!
                found_buffer_idx = i;
                break;
            }
        }

        if (found_buffer_idx == -1) {
            // We did not find a buffer... it is a new frame!
            total_frames_received++;

            FrameBuffer our_buffer = buffers[next_buffer];

            if (our_buffer.timestamp != 0) {
                // It is not a new buffer... it already has a frame in it.

                if (our_buffer.remaining_sections == 0) {
                } else {
                    System.out.println("> Dropped frame with timestamp " + our_buffer.timestamp + ", dropped perc " + total_frames_dropped / (double) total_frames_received);
                    total_frames_dropped++;
                }
            }

            our_buffer.timestamp = timestamp;
            our_buffer.remaining_sections = section_count - 1;
            System.arraycopy(frame_data, 0, our_buffer.internal_buffer, section_id*PacketCamera.MAX_FRAME_DATA_SIZE, frame_data_size);
            our_buffer.buffer_size = frame_data_size;

            if (our_buffer.remaining_sections == 0) {
                our_buffer.push();
            }

            // Reset our next_buffer.
            next_buffer = (next_buffer + 1) % maxSize;
        } else {
            // We found our buffer, let's update it!

            FrameBuffer our_buffer = buffers[found_buffer_idx];

            our_buffer.remaining_sections--;
            System.arraycopy(frame_data, 0, our_buffer.internal_buffer, section_id*PacketCamera.MAX_FRAME_DATA_SIZE, frame_data_size);
            our_buffer.buffer_size += frame_data_size;

            if (our_buffer.remaining_sections == 0) {
                our_buffer.push();
            }
        }
    }
}

public class PacketCameraHandler implements PacketHandler {
    private FrameBufferContainer frameBufferContainer = new FrameBufferContainer(10);

    @Override
    public void handle(Packet packet) {
        try {
            frameBufferContainer.update_buffer((PacketCamera) packet);
        } catch (IllegalArgumentException e) {
            System.out.println("Unable to add section to potential frame " + e.getMessage());
        }
    }
}