package com.github.zeldazach.binghamtonrover.networking;

import java.nio.ByteBuffer;

public class PacketControl extends Packet
{
    public enum MovementDirection
    {
        STOP,
        FORWARD,
        LEFT,
        RIGHT,
        BACKWARD
    }

    private MovementDirection direction;

    PacketControl(MovementDirection _direction)
    {
        super((byte) 1, 1);
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
        direction = MovementDirection.values()[buff.get()];
    }
}