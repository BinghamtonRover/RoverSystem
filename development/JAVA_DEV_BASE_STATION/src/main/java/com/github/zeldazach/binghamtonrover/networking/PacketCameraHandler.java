package com.github.zeldazach.binghamtonrover.networking;

import com.github.zeldazach.binghamtonrover.gui.DisplayApplication;
import javafx.application.Platform;

import java.nio.Buffer;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;

public class PacketCameraHandler implements PacketHandler {
    public static class PotentialFrame<K extends Integer, V extends Buffer> extends HashMap {
        private final int maxSections;

        PotentialFrame(int _maxSections, int initialCapacity) {
            super(initialCapacity);
            maxSections = _maxSections;
        }

        PotentialFrame(int _maxSections) {
            this(_maxSections, 16); // see oracle default initial values
        }

        synchronized void addSection(int section, ByteBuffer sectionBuffer) {
            if (section >= maxSections) {
                throw new IllegalArgumentException("section index out of bounds for this PotentialFrame");
            } /*else if (this.containsKey(section) ? what should we do in this case? */ else {
                put(section, sectionBuffer);
            }
            if (this.complete()) {
                // this potential frame has just been completed, notify the camera view watcher
                Platform.runLater(() -> {
                    DisplayApplication app = DisplayApplication.INSTANCE;
                    if (app != null) {
                        app.cameraViewWatcher.notify();
                    }
                });
            }
        }

        synchronized boolean complete() { return this.size() == this.maxSections; }

        public int getMaxSections() { return maxSections; }
    }

    public static class FrameBuffer<K extends Number, V extends PotentialFrame> extends HashMap {
        private final int maxFrameBuffer = 5; // should be sufficiently large to avoid issues getting individual sections out of order

        synchronized void setFrameSection(int timestamp, int sectionIndex, int sectionCount, ByteBuffer sectionBuff) {
            boolean conditionalContains = true;
            // if we are current'y holding maxFrameBuffer potential frames,
            // and the currently received frame isn't one of them,
            if (this.size() == this.maxFrameBuffer && !this.containsKey(timestamp)) {
                // and is a newly timed frame, and not one from earlier
                if (timestamp > ((Integer) Collections.max(keySet()))) {
                    // remove the oldest potential frame to make room for this new one
                    this.remove(Collections.min(keySet()));
                    conditionalContains = false;
                } else {
                    // well we don't have this one, but it's not new, so we don't care
                    return;
                }
            }
            // if we definitely don't have it, short circuit and add a new one
            // otherwise, make sure we don't have it before adding a new one
            if (!conditionalContains || !this.containsKey(timestamp)) {
                // this technically subclasses potential frame, but it's fine because we upcast internally
                put(timestamp, new PotentialFrame<Integer, ByteBuffer>(sectionCount) {{
                    addSection(sectionIndex, sectionBuff);
                }});
            } else {
                // we got it, downcast and update it with the new section
                ((PotentialFrame) this.get(timestamp)).addSection(sectionIndex, sectionBuff);
            }
        }

        synchronized void setFrameSection(PacketCamera packet) {
            setFrameSection(
                    packet.getHeader().getTimestamp(),
                    packet.getSectionIndex(),
                    packet.getSectionCount(),
                    packet.getSectionData());
        }

        public synchronized PotentialFrame getMostRecentCompletedFrame() {
            Integer largestTimestamp = 0;
            PotentialFrame completedFrame, currentFrame;
            completedFrame = null;
            for (Object key: keySet()) {
                if ((Integer) key > largestTimestamp) {
                    largestTimestamp = (Integer) key;
                    currentFrame = (PotentialFrame) get(largestTimestamp);
                    if (currentFrame.complete()) completedFrame = currentFrame;
                }
            }
            return completedFrame;
        }

    }
    private static final FrameBuffer<Integer, PotentialFrame<Integer, ByteBuffer>> framesToCreate = new FrameBuffer<>();

    public static FrameBuffer<Integer, PotentialFrame<Integer, ByteBuffer>> getFramesToCreate() { return framesToCreate; }

    @Override
    public void handle(Packet packet) {
        try {
            framesToCreate.setFrameSection((PacketCamera) packet);
        } catch (IllegalArgumentException e) {
            System.out.println("Unable to add section to potential frame " + e.getMessage());
        }
    }
}