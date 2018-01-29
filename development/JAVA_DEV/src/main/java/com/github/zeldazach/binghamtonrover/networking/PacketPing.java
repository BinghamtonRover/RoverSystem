package com.github.zeldazach.binghamtonrover.networking;

import java.nio.ByteBuffer;

public class PacketPing extends Packet {
    public static enum Direction {
        PING,
        PONG
    }

    public Direction direction;

    public PacketPing(Direction direction) {
        super((byte) 0, 1);

        this.direction = direction;
    }

    @Override
    public void writeToBuffer(ByteBuffer buff) {
        buff.put((byte) direction.ordinal());
    }

    @Override
    public void readFromBuffer(ByteBuffer buff) {
        direction = Direction.values()[buff.get()];
    }
}
