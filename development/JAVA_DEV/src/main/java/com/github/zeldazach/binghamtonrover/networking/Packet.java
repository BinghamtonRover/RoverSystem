package com.github.zeldazach.binghamtonrover.networking;

import java.net.DatagramPacket;
import java.net.InetAddress;
import java.nio.ByteBuffer;
import java.util.HashMap;
import java.util.Map;

class PacketHeader {
    public static final int SIZE = 5; // The current size of a packet header.

    private char version; // equivalent to unsigned short
    private byte type;
    private char timestamp; // equivalent to unsigned short

    PacketHeader(char version, byte type, char timestamp) {
        this.version = version;
        this.type = type;
        this.timestamp = timestamp;
    }

    public PacketHeader() {

    }

    public int getVersion() {
        return version;
    }

    public int getType() {
        return type & 0xFF; // if the type if over 127 we get an underflow
    }

    public int getTimestamp() {
        return timestamp;
    }

    public void writeToBuffer(ByteBuffer buff) {
        buff.putShort((short) version);
        buff.put(type);
        buff.putShort((short) timestamp);
    }

    public void readFromBuffer(ByteBuffer buff) {
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
public abstract class Packet {
    private final static int MAX_SIZE = 6; // max size known for a packet

    private byte type;
    private int size;

    /**
     * Sets the type of the packet and its length.
     * Since Packet is abstract, this will be implemented by its children.
     * @param type The packet type.
     * @param size The packet size.
     */
    public Packet(byte type, int size) {
        this.type = type;
        this.size = size;
    }

    public Packet() {

    }

    public byte getType() {
        return type;
    }

    public int getSize() {
        return size;
    }

    /**
     * This is called when the packet must be written to the network.
     * Children must implement this. Assume that the buffer has
     * enough space to hold the entire packet.
     * @param buff The buffer in which the packet must be written.
     */
    public abstract void writeToBuffer(ByteBuffer buff);
    public abstract void readFromBuffer(ByteBuffer buff);

    public static DatagramPacket makeReceivingPacket() {
        return makeReceivingPacket(MAX_SIZE);
    }

    public static DatagramPacket makeReceivingPacket(int readAmt) {
        return new DatagramPacket(new byte[MAX_SIZE], readAmt);
    }
}