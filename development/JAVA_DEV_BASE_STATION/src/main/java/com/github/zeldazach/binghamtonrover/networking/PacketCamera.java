package com.github.zeldazach.binghamtonrover.networking;

import java.nio.ByteBuffer;

public class PacketCamera extends Packet {
    private int sectionIndex, sectionCount, sectionSize;
    private ByteBuffer sectionData;

    @Override
    public void writeToBuffer(ByteBuffer buff) {
        throw new IllegalStateException("BASE STATION MUST NOT SEND CAMERA PACKETS!");
    }

    @Override
    public void readFromBuffer(ByteBuffer buff) {
        // Skip the length.
        sectionIndex = buff.get();
        sectionCount = buff.get();
        sectionSize = buff.getShort();
        byte[] sectionArray = new byte[buff.limit() - buff.position()];
        buff.get(sectionArray);
        sectionData = ByteBuffer.wrap(sectionArray);
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

    public ByteBuffer getSectionData() {
        return sectionData;
    }
}
