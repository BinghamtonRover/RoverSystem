package BinghamtonRover.Networking;

import java.net.DatagramPacket;

public interface PacketHandler {
    void handle(DatagramPacket packet);
}