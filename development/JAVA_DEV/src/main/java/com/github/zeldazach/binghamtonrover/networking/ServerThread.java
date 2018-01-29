package com.github.zeldazach.binghamtonrover.networking;

import java.io.IOException;
import java.net.DatagramPacket;
import java.nio.ByteBuffer;

public abstract class ServerThread extends Thread {
    protected Manager serverManager;

    protected ServerThread(String name, Manager m) {
        super(name);
        this.serverManager = m;
    }
}

class ServerReceiver extends ServerThread {
    ServerReceiver(Manager m) {
        super("UDP Receiver", m);

        // So the program actually stops...
        setDaemon(true);
    }

    public void run() {
        DatagramPacket datagramPacket;

        while (!this.serverManager.socket.isClosed()) {
            datagramPacket = Packet.makeReceivingPacket();

            try {
                this.serverManager.socket.receive(datagramPacket);
            } catch (IOException e) {
                e.printStackTrace();
                continue;
            }

            serverManager.handlePacket(datagramPacket);
        }
    }
}