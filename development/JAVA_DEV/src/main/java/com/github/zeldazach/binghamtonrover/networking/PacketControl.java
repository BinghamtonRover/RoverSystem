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

    /*
    TODO: THe point of this method is to make an appropriate packet object from
    a ByteBuffer... but since we won't ever do this for this packet type it's
    unnecessary, maybe we should throw an exception instead?
    */
    @Override
    public void readFromBuffer(ByteBuffer buff)
    {
        direction = MovementDirection.values()[buff.get()];
    }
}