package com.github.zeldazach.binghamtonrover.networking;

import java.net.DatagramPacket;
import java.net.InetAddress;
import java.nio.ByteBuffer;
import java.util.HashMap;
import java.util.Map;

class PacketHeader {
    private final char version; // equivalent to unsigned short
    private final byte type;
    private final char timestamp; // equivalent to unsigned short

    PacketHeader(char version, byte type, char timestamp) {
        this.version = version;
        this.type = type;
        this.timestamp = timestamp;
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

    public byte[] getHeaderBuffer() {
        byte[] buff = new byte[5];

        // Wrap the byte array to make serializing easy.
        ByteBuffer loWrapper = ByteBuffer.wrap(buff);
        loWrapper.putShort((short) version);
        loWrapper.put(type);
        loWrapper.putShort((short) timestamp);

        return buff;
    }
}

public abstract class Packet {
    private final PacketHeader packetHeader;
    private final byte[] packetData;
    private final static int MAX_SIZE = 6; // max size known for a packet

    public Packet(char version, byte type, char timestamp, byte[] packetData) {
        this(new PacketHeader(version, type, timestamp), packetData);
    }

    public Packet(PacketHeader packetHeader, byte[] packetData) {
        this.packetHeader = packetHeader;
        this.packetData = packetData;
    }

    public static DatagramPacket makeReceivingPacket() {
        return makeReceivingPacket(MAX_SIZE);
    }

    public static DatagramPacket makeReceivingPacket(int readAmt) {
        return new DatagramPacket(new byte[MAX_SIZE], readAmt);
    }

    public DatagramPacket toDatagramPacket(InetAddress addr, int port) {
        byte[] headerBuffer = packetHeader.getHeaderBuffer();
        byte[] fullPacket = new byte[headerBuffer.length + packetData.length];
        System.arraycopy(headerBuffer, 0, fullPacket, 0, headerBuffer.length);
        System.arraycopy(packetData, 0, fullPacket, headerBuffer.length, fullPacket.length);
        return new DatagramPacket(fullPacket, fullPacket.length, addr, port);
    }
}

class ControlPacket extends Packet {
    public ControlPacket(char version, char timestamp, byte[] packetData) {
        super(version, (byte) 1, timestamp, packetData);
    }
}

class StopPacket extends ControlPacket {
    private static byte[] data = {0};
    public StopPacket(char version, char timestamp) {
        super(version, timestamp, data);
    }
}

class ForwardPacket extends ControlPacket {
    private static byte[] data = {1};
    public ForwardPacket(char version, char timestamp) {
        super(version, timestamp, data);
    }
}

class LeftPacket extends ControlPacket {
    private static byte[] data = {2};
    public LeftPacket(char version, char timestamp) {
        super(version, timestamp, data);
    }
}

class RightPacket extends ControlPacket {
    private static byte[] data = {3};
    public RightPacket(char version, char timestamp) {
        super(version, timestamp, data);
    }
}