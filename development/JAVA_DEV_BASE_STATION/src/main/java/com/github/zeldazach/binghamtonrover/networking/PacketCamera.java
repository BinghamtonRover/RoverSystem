package com.github.zeldazach.binghamtonrover.networking;

import java.nio.ByteBuffer;

public class PacketCamera extends Packet {
    // Maximum size of frame data included within a single packet.
    public static final int MAX_FRAME_DATA_SIZE = 40000;

    private int sectionIndex, sectionCount, sectionSize;
    private byte[] sectionData;

    @Override
    public void writeToBuffer(ByteBuffer buff) {
        throw new IllegalStateException("BASE STATION MUST NOT SEND CAMERA PACKETS!");
    }

    @Override
    public void readFromBuffer(ByteBuffer buff) {
        // This will fail if the sectionCount or sectionIndex is greater than 127.
        // That will never happen due to the side of our frames, but it is possible
        // for this to fail theoretically.
        // TODO: Switch this to properly handle the unsigned type.
        // We might want to create a utility class which handles this reliably for us.

        sectionIndex = buff.get();
        sectionCount = buff.get();
        // Char just so happens to be the only unsigned type in Java,
        // so we take advantage of that here.
        sectionSize = buff.getChar();
        sectionData = new byte[buff.limit() - buff.position()];
        buff.get(sectionData);
    }

    public int getSectionIndex() {
        return sectionIndex;
    }

    public int getSectionCount() {
        return sectionCount;
    }

    public int getSectionSize() {
        return sectionSize;
    }

    public byte[] getSectionData() {
        return sectionData;
    }
}
