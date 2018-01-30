package com.github.zeldazach.binghamtonrover.networking;

import java.nio.ByteBuffer;

public class PacketPing extends Packet
{
    public enum Direction
    {
        PING,
        PONG
    }

    private Direction direction;

    public PacketPing(Direction _direction)
    {
        super((byte) 0, 1);
        direction = _direction;
    }

    @Override
    public void writeToBuffer(ByteBuffer buff)
    {
        buff.put((byte) direction.ordinal());
    }

    @Override
    public void readFromBuffer(ByteBuffer buff)
    {
        direction = Direction.values()[buff.get()];
    }
}
