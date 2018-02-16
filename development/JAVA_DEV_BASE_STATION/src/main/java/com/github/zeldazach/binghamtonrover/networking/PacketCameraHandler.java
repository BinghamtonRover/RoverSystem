package com.github.zeldazach.binghamtonrover.networking;

import com.github.zeldazach.binghamtonrover.gui.DisplayApplication;
import javafx.application.Platform;
import javafx.scene.image.Image;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.Arrays;

class FrameBuffer {
    private ByteBuffer[][] internalBuffer;

    private int[] timestamps;

    private int size;

    FrameBuffer(int size) {
        this.size = size;
        timestamps = new int[size];
        internalBuffer = new ByteBuffer[5][0]; // internal arrays are changed anyway
    }

    private void updateFrame(PacketCamera packet, int index) {
        internalBuffer[index][packet.getSectionIndex()] = packet.getSectionData();
        if (!Arrays.asList(internalBuffer[index]).contains(null)) {
            // frame is complete
            ByteArrayOutputStream baos = new ByteArrayOutputStream();
            try {
                for (ByteBuffer buff: internalBuffer[index]) {
                    baos.write(buff.array());
                }
            } catch (IOException ex) {
                System.err.println("Couldn't concat complete frame TIMESTAMP: " + timestamps[index]);
                return;
            }
            Image image = new Image(new ByteArrayInputStream(baos.toByteArray()));
            Platform.runLater(() -> {
                if (DisplayApplication.INSTANCE != null) {
                    DisplayApplication.INSTANCE.getCameraImageView().setImage(image);
                }
            });
        }
    }

    private void shiftBuffer(PacketCamera packet) {
        int[] tempTimestamps = new int[size];
        ByteBuffer[][] tempInternalBuffer = new ByteBuffer[size][0];
        System.arraycopy(timestamps, 1, tempTimestamps, 0, size - 1);
        System.arraycopy(internalBuffer, 1, tempInternalBuffer, 0, size - 1);
        timestamps[size - 1] = packet.getHeader().getTimestamp();
        internalBuffer[size - 1] = new ByteBuffer[packet.getSectionCount()];
        timestamps = tempTimestamps;
        internalBuffer = tempInternalBuffer;
    }

    public void update(PacketCamera packet) {
        boolean timestampWithin = false;
        for (int i = 0; i < timestamps.length; i++) {
            if (timestamps[i] == 0) {
                timestamps[i] = packet.getHeader().getTimestamp();
                internalBuffer[i] = new ByteBuffer[packet.getSectionCount()];
                timestampWithin = true;
                updateFrame(packet, i);
            } else if (timestamps[i] == packet.getHeader().getTimestamp()) {
                updateFrame(packet, i);
                timestampWithin = true;
            }
        }
        if (!timestampWithin) {
            shiftBuffer(packet);
            updateFrame(packet, size - 1);
        }
    }
}


public class PacketCameraHandler implements PacketHandler {
    private FrameBuffer frameBuffer = new FrameBuffer(5);

    @Override
    public void handle(Packet packet) {
        try {
            frameBuffer.update((PacketCamera) packet);
        } catch (IllegalArgumentException e) {
            System.out.println("Unable to add section to potential frame " + e.getMessage());
        }
    }
}