package BinghamtonRover.Networking;

import java.io.IOException;
import java.net.DatagramPacket;

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
    }

    public void run() {
        byte[] data;
        int version;
        int timestamp, expectedTimstamp;
        DatagramPacket datagramPacket;

        while (!this.serverManager.socket.isClosed()) {
            datagramPacket = Packet.makeReceivingPacket();

            try {
                this.serverManager.socket.receive(datagramPacket);
            } catch (IOException e) {
                e.printStackTrace();
                continue;
            }

            data = datagramPacket.getData();
            version = this.getVersionFromData(data);
            timestamp = this.getTimestampFromData(data);
            expectedTimstamp = serverManager.getLastReceiveTimestamp();
            if (version != this.serverManager.version) {
                this.outputVersionMismatch(version);
            } else if (timestamp < expectedTimstamp) {
                this.outputTimestampMismatch(timestamp, expectedTimstamp);
            } else {
                this.handle(datagramPacket, data[2]);
            }

            try {
                Thread.sleep(serverManager.clockInterval);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    private void outputVersionMismatch(int version) {
        int correctVersion = serverManager.version;
        System.out.println(
                "Packet Received with incorrect version. Expected " + correctVersion + " but got " + version);
    }

    private void outputTimestampMismatch(int actual, int expected) {
        System.out.println(
                "Packet Received with outdated timestamp. Expected at least" + expected + " but got " + actual);
    }

    private void handle(DatagramPacket datagramPacket, byte type) {
        serverManager.incrementReceiveTimestamp();
        serverManager.getHandler(type).handle(datagramPacket);
    }
    private int getVersionFromData(byte[] data) {
        return (int)(
                (((char)data[0]) << 8) | (data[1] & 0xFF)
        );
    }

    private int getTimestampFromData(byte[] data) {
        return (int)(
                (((char)data[3]) << 8) | (data[4] & 0xFF)
        );
    }
}

class ServerSender extends ServerThread {
    ServerSender(Manager m) {
        super("ServerSender", m);
    }

    public void run() {
        DatagramPacket packet;
        while (true) {
            packet = serverManager.getLatestPacket().toDatagramPacket(
                    serverManager.getAddress(), serverManager.getPort());

            try {
                serverManager.getSocket().send(packet);
            } catch (IOException e) {
                e.printStackTrace();
            }

            serverManager.incrementSendTimestamp();

            try {
                Thread.sleep(serverManager.clockInterval);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
