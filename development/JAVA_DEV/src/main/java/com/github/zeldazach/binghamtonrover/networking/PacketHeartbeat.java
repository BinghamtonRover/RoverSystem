package com.github.zeldazach.binghamtonrover.networking;

import java.nio.ByteBuffer;

public class PacketPing extends Packet
{
    private static int MAX_SIZE = 6;
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

    // See other comment about readFromBuffer
    @Override
    public void readFromBuffer(ByteBuffer buff)
    {
        direction = Direction.values()[buff.get()];
    }
}
