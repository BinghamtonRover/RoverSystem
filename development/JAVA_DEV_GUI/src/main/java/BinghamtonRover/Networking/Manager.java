package BinghamtonRover.Networking;

import java.net.*;
import java.util.HashMap;
import java.util.Map;

class ManagerException extends Exception {
    public ManagerException(String message) {
        super(message);
    }
}
class AlreadyStarted extends ManagerException {
    public AlreadyStarted() {
        super("Manager already started");
    }
}
class AlreadyClosed extends ManagerException {
    public AlreadyClosed() {
        super("Manager already closed");
    }
}

public class Manager {
    private InetAddress address;
    private int port;
    int clockInterval;
    DatagramSocket socket;
    private Packet latestPacket;
    private ServerSender sender;
    private ServerReceiver receiver;
    private boolean started = false;
    private boolean closed = false;
    private char sendTimestamp = 0;
    private char receiveTimestamp = 0;
    final char version = 1;
    private Map<Integer, PacketHandler> handlers = new HashMap<Integer, PacketHandler>();
    private Map<Integer, Class<Packet>> packetsByType = new HashMap<Integer, Class<Packet>>() {{
        put(1, ControlPacket);
    }};

    public Manager() throws SocketException, UnknownHostException {
        this("0.0.0.0", 8080, 5);
    }

    /**
     * Construct a Manager for sending and receiving packets
     * @param addr the address to bind to
     * @param port the port to bind on
     * @param clockInterval time in milliseconds between each packet read and send
     * @throws SocketException when the socket can't be bound
     * @throws UnknownHostException when the address is invalid
     */
    public Manager(String addr, int port, int clockInterval) throws SocketException, UnknownHostException {
        this.address = Inet4Address.getByName(addr);
        this.port = port;
        this.clockInterval = clockInterval;
        this.socket = new DatagramSocket(this.port, this.address);
        this.sender = new ServerSender(this);
        this.receiver = new ServerReceiver(this);
    }

    public void startServer() throws AlreadyStarted, SocketException {
        if (started) throw new AlreadyStarted();
        if (!socket.isBound()) socket.bind(new InetSocketAddress(address, port));
        sender.start();
        receiver.start();
        started = true;
        closed = false;
    }

    public void closeServer() throws AlreadyClosed, InterruptedException {
        if (closed) throw new AlreadyClosed();
        sender.join();
        receiver.join();
        socket.close();
        started = false;
        closed = true;
    }

    public void setHandler(int packetType, PacketHandler handler) {
        handlers.put(packetType, handler);
    }

    public PacketHandler getHandler(int type) { return handlers.get(type); }

    public Packet constructPacket(int type, byte[] packetData) {
        PacketHeader header = new PacketHeader(version, (byte) type, getLastSendTimestamp());
        return constructPacket(header, packetData);
    }

    public Packet constructPacket(PacketHeader packetHeader, byte[] packetData) {
        return new ((packetsByType.get(packetHeader.getType()))(packetHeader, packetData));
    }

    PacketHandler getHandler(byte type) { return getHandler(type & 0xFF); }

    public DatagramSocket getSocket() {
        return socket;
    }

    public InetAddress getAddress() {
        return address;
    }

    public int getPort() {
        return port;
    }

    public synchronized void sendPacket(Packet p) {
        this.latestPacket = p;
    }

    synchronized Packet getLatestPacket() { return latestPacket; }

    synchronized void incrementReceiveTimestamp() { receiveTimestamp++; }

    synchronized void incrementSendTimestamp() { sendTimestamp++; }

    private synchronized char getLastSendTimestamp() { return sendTimestamp; }

    synchronized int getLastReceiveTimestamp() { return receiveTimestamp; }

    public static void main(String[] args) {
        try {
            Manager my_manager = new Manager();
            my_manager.startServer();
            my_manager.closeServer();
        } catch (SocketException | UnknownHostException | AlreadyStarted | AlreadyClosed | InterruptedException e) {
            e.printStackTrace();
        }
    }
}