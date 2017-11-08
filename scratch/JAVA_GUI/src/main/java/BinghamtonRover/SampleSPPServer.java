


import java.io.*;
import javax.bluetooth.*;
import javax.microedition.io.*;

/**
 * Class that implements an SPP Server which accepts single TXT file
 * from an SPP client.
 */
public class SampleSPPServer {

    //start server
    private void startServer() throws IOException{
        int filesize=6022386*16; // filesize temporary hardcoded
        long start = System.currentTimeMillis();
        int bytesRead;
        int current = 0;

        //Create a UUID for SPP
        UUID uuid = new UUID("1101", true);
        //Create the servicve url
        String connectionString = "btspp://localhost:" + uuid +";name=Sample SPP Server";

        //open server url
        StreamConnectionNotifier streamConnNotifier = (StreamConnectionNotifier)Connector.open( connectionString );

        //Wait for client connection
        System.out.println("\nServer Started. Waiting for clients to connect...");
        StreamConnection connection=streamConnNotifier.acceptAndOpen();

        RemoteDevice dev = RemoteDevice.getRemoteDevice(connection);
        System.out.println("Remote device address: "+dev.getBluetoothAddress());
        System.out.println("Remote device name: "+dev.getFriendlyName(true));

        //get file from spp client
        InputStream is=connection.openInputStream();

        // receive file
        byte [] mybytearray  = new byte [filesize];
        FileOutputStream fos = new FileOutputStream("source-copy.txt");
        BufferedOutputStream bos = new BufferedOutputStream(fos);
        bytesRead = is.read(mybytearray,0,mybytearray.length);
        current = bytesRead;


        do {
            bytesRead =
                    is.read(mybytearray, current, (mybytearray.length-current));
            if(bytesRead >= 0) current += bytesRead;
        } while(bytesRead > -1);

        bos.write(mybytearray, 0 , current);
        bos.flush();
        long end = System.currentTimeMillis();
        System.out.println("Transmition time : "+(end-start));
        bos.close();
        fos.close();
        is.close();
        streamConnNotifier.close();

    }


    public static void main(String[] args) throws IOException {

        //display local device address and name
        LocalDevice localDevice = LocalDevice.getLocalDevice();
        System.out.println("My BlueTooth Device Address: "+localDevice.getBluetoothAddress());
        System.out.println("My BlueTooth Device Name: "+localDevice.getFriendlyName());

        SampleSPPServer sampleSPPServer=new SampleSPPServer();
        sampleSPPServer.startServer();

    }
}