package com.github.zeldazach.binghamtonrover.networking;

import java.net.DatagramPacket;
import java.nio.ByteBuffer;
import java.util.Optional;

class PacketHeader
{
    public static final int SIZE = 5; // The current size of a packet header.

    private char version; // equivalent to unsigned short
    private byte type;
    private char timestamp; // equivalent to unsigned short

    PacketHeader(char _version, byte _type, char _timestamp)
    {
        version = _version;
        type = _type;
        timestamp = _timestamp;
    }

    PacketHeader() {}

    public int getVersion()
    {
        return version;
    }

    public int getType()
    {
        return type & 0xFF; // if the type if over 127 we get an underflow
    }

    public int getTimestamp()
    {
        return timestamp;
    }

    public void writeToBuffer(ByteBuffer buff)
    {
        buff.putShort((short) version);
        buff.put(type);
        buff.putShort((short) timestamp);
    }

    public void readFromBuffer(ByteBuffer buff)
    {
        version = buff.getChar();
        type = buff.get();
        timestamp = buff.getChar();
    }
}

/**
 * It is important that subclasses of Packet set up their
 * type and size if they are going to be sent over the network!
 * They should call Packet's constructor in their constructors.
 * This is temporary, until I have time to design a better
 * system.
 */
public abstract class Packet
{
    // This value is magic. This is the maximum size of a full camera packet.
    private final static int MAX_SIZE = 40009;

    private byte type;
    private int size;
    private Optional<Manager> manager = Optional.empty();
    private PacketHeader header;

    /**
     * Sets the type of the packet and its length.
     * Since Packet is abstract, this will be implemented by its children.
     * @param _type The packet type.
     * @param _size The packet size.
     */
    Packet(byte _type, int _size)
    {
        type = _type;
        size = _size;
    }

    Packet() {}

    public byte getType()
    {
        return type;
    }

    public int getSize()
    {
        return size;
    }

    public Optional<Manager> getManager() {
        return manager;
    }

    private void setManager(Optional<Manager> manager) {
        this.manager = manager;
    }

    public void setManager(Manager manager) {
        setManager(Optional.of(manager));
    }

    /**
     * This is called when the packet must be written to the network.
     * Children must implement this. Assume that the buffer has
     * enough space to hold the entire packet.
     * @param buff The buffer in which the packet must be written.
     */
    public abstract void writeToBuffer(ByteBuffer buff);
    public abstract void readFromBuffer(ByteBuffer buff);

    public static DatagramPacket makeReceivingPacket()
    {
        return makeReceivingPacket(MAX_SIZE);
    }

    private static DatagramPacket makeReceivingPacket(int readAmt)
    {
        return new DatagramPacket(new byte[MAX_SIZE], readAmt);
    }

    public void setHeader(PacketHeader header) {
        this.header = header;
    }

    public PacketHeader getHeader() {
        return header;
    }
}