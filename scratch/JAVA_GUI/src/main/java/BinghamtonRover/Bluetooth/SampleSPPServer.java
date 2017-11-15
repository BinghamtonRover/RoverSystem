package BinghamtonRover.Bluetooth;

import java.io.*;
import javax.bluetooth.*;
import javax.microedition.io.*;

/**
 * A thread that opens an SPP connection, awaits client requests, accepts
 * incoming messages, and sends a response.
 */
public class SampleSPPServer implements Runnable
{

    private static final String gsSERVICE_NAME_SSP = "The SSP Server";

    public void run()
    {
        try
        {
            StreamConnection connection;
            DataOutputStream outputStream = null;
            DataInputStream inputStream = null;

            try
            {
                //Generate a new UUID and get the local device information
                final UUID uuid = new UUID("1101", false);
                final LocalDevice local = LocalDevice.getLocalDevice();

                //use the updateStatus method to print to console for now
                //Later will use updateStatus to update the GUI
                updateStatus("[SERVER] Device Address: " + local.getBluetoothAddress());
                updateStatus("[SERVER] Device Name: " + local.getFriendlyName());
                updateStatus("[SERVER] Listening for Client...");

                // Open a connection and wait for client requests
                // spawn a thread for each client connection in order to listen to multiple client simultaneously
                final StreamConnectionNotifier service =
                        (StreamConnectionNotifier) Connector.open("btspp://localhost:" + uuid + ";name=" + gsSERVICE_NAME_SSP);
                connection = service.acceptAndOpen();
                updateStatus("[SERVER] SPP session created");

                inputStream = connection.openDataInputStream();
                outputStream = connection.openDataOutputStream();
                while(true) {

                    // Read a message
                    //a buffer byte array used to temporarily hold the input, 1 kb should be enough for short text input
                    final byte[] buffer = new byte[1024];
                    //get the numbers of bytes of the input and then save the input to buffer
                    final int readBytes = inputStream.read(buffer);
                    //build a string of the receiving message from the buffer
                    final String receivedMessage = new String(buffer, 0, readBytes);
                    //Print the received message
                    updateStatus("[SERVER] Message received: " + receivedMessage);

                    // Send a message
                    final String message = "\nJSR-82 SERVER says hello!";
                    updateStatus("[SERVER] Sending message....");
                    //Send out the message to the other end of the connection
                    outputStream.write(message.getBytes());
                    outputStream.flush();
                }
            }

            finally
            {
                outputStream.close();
                inputStream.close();
                updateStatus("[SERVER] SPP session closed");
            }
        }
        catch (final IOException ioe)
        {
            //BluetoothJSR82Demo.errorDialog(ioe.toString());
            ioe.printStackTrace();
        }
    }

    private void updateStatus(String asMessage)
    {
        System.out.println(asMessage);
    }


    /**
     * Here is a thread class that will be created every time a connection is made
     * This thread is expected to handle the communication between the client and server
     *
     * when the server received a connection service, it will establish a connection and
     * instantiate a SeverClientConnection thread that listens to the message the client
     * sent and respond with, for now, a response string
     * Not yet implemented
     */
    class ServerClientConnection extends Thread{

    }



    public static void main(String[] args) throws IOException {

        //display local device address and name
        /**
         * Will be displayed in the sampleSSPServer class
         */
//        LocalDevice localDevice = LocalDevice.getLocalDevice();
//        System.out.println("My BlueTooth Device Address: "+localDevice.getBluetoothAddress());
//        System.out.println("My BlueTooth Device Name: "+localDevice.getFriendlyName());

        SampleSPPServer sampleSPPServer = new SampleSPPServer();
        sampleSPPServer.run();

    }
}