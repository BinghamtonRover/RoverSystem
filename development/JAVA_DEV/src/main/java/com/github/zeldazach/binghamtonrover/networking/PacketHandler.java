package com.github.zeldazach.binghamtonrover.networking;

import java.net.DatagramPacket;

public interface PacketHandler {
    void handle(DatagramPacket packet);
}