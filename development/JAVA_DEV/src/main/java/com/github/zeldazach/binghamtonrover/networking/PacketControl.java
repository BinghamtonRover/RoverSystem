package com.github.zeldazach.binghamtonrover.networking;

import java.nio.ByteBuffer;

public class PacketControl extends Packet {
    public static enum MovementDirection {
        STOP,
        FORWARD,
        LEFT,
        RIGHT,
        BACKWARD
    }

    public MovementDirection direction;

    public PacketControl(MovementDirection direction) {
        super((byte) 1, 1);

        this.direction = direction;
    }

    @Override
    public void writeToBuffer(ByteBuffer buff) {
        buff.put((byte) direction.ordinal());
    }

    @Override
    public void readFromBuffer(ByteBuffer buff) {
        direction = MovementDirection.values()[buff.get()];
    }
}