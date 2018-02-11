package com.github.zeldazach.binghamtonrover.networking;

import java.io.IOException;
import java.net.DatagramPacket;
import java.nio.ByteBuffer;

abstract class ServerThread extends Thread
{
    Manager serverManager;

    ServerThread(String name, Manager m)
    {
        super(name);
        serverManager = m;
    }
}

class ServerReceiver extends ServerThread
{
    ServerReceiver(Manager m)
    {
        super("UDP Receiver", m);

        // So the program actually stops...
        setDaemon(true);
    }

    public void run()
    {
        DatagramPacket datagramPacket;

        while (!serverManager.getSocket().isClosed())
        {
            datagramPacket = Packet.makeReceivingPacket();

            // TODO: Explain how this is UDP protocol
            try
            {
                serverManager.getSocket().receive(datagramPacket);
            }
            catch (IOException e)
            {
                e.printStackTrace();
                continue;
            }


            serverManager.handlePacket(datagramPacket);
        }
    }
}