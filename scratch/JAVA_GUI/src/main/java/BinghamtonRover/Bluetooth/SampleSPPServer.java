package BinghamtonRover.Bluetooth;

import javafx.fxml.FXML;

import javax.bluetooth.BluetoothStateException;
import javax.bluetooth.LocalDevice;
import javax.bluetooth.UUID;
import javax.microedition.io.Connector;
import javax.microedition.io.StreamConnection;
import javax.microedition.io.StreamConnectionNotifier;

import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;

import java.sql.Timestamp;
import java.time.Instant;


/**
 * A thread that opens an SPP connection, awaits client requests, accepts
 * incoming messages, and sends a response.
 */
public class SampleSPPServer implements Runnable {

    private static final String gsSERVICE_NAME_SSP = "The SSP Server";

    public void run() {
            try {
                //Generate a new UUID and get the local device information
                final UUID uuid = new UUID("1101", false);
                final LocalDevice local = LocalDevice.getLocalDevice();

                //use the updateStatus method to print to console for now
                //Later will use updateStatus to update the GUI
                updateStatus("[SERVER] Device Address: " + local.getBluetoothAddress());
                updateStatus("[SERVER] Device Name: " + local.getFriendlyName());
                updateStatus("[SERVER] Listening for Client...");

                final StreamConnectionNotifier service = (StreamConnectionNotifier) Connector.open("btspp://localhost:" + uuid + ";name=" + gsSERVICE_NAME_SSP);

                while(true)
                {
                    // Open a connection and wait for client requests
                    // spawn a thread for each client connection in order to listen to multiple client simultaneously
                    StreamConnection loConnection = service.acceptAndOpen();

                    //Create a new thread to handle each session
                    updateStatus("[SERVER] SPP session created");
                    ServerClientConnection loSCC = new ServerClientConnection(loConnection);
                    loSCC.run();
                }
            }
            catch (BluetoothStateException e)
            {
                e.printStackTrace();
            } catch (IOException e) {
                e.printStackTrace();
            }
    }

    private void updateStatus(String asMessage) {
        System.out.println(asMessage);
//        coMessageBoard.appendText(asMessage);
    }

    /**
     * Here is a thread class that will be created every time a connection is made
     * This thread is expected to handle the communication between the client and server
     * <p>
     * when the server received a connection request, it will establish a connection and
     * instantiate a SeverClientConnection thread that listens to the message the client
     * sent and respond with, for now, a response string
     * Not yet implemented
     */
    class ServerClientConnection extends Thread {

        private StreamConnection coConnection;
        private LocalDevice coDevice;

        public ServerClientConnection(StreamConnection aoConnection)
        {
            coConnection = aoConnection;
        }

        public void run()
        {

            //Declare data IO stream
            DataOutputStream outputStream = null;
            DataInputStream inputStream = null;

            try
            {
                outputStream = coConnection.openDataOutputStream();
                inputStream = coConnection.openDataInputStream();

                //Instantiate a timestamp to keep time of connection session time.
                Timestamp loTimestamp = new Timestamp(System.currentTimeMillis());
                updateStatus("Session started at: " + loTimestamp);



                long lnSessionInactiveTime = System.currentTimeMillis() - loTimestamp.getTime();
                while(lnSessionInactiveTime < 10000)
                {
                    String lsMessage =  "Server Says Hello";
                    String lsReceivedMessage;
//
                    // Read a message
                    //a buffer byte array used to temporarily hold the input, 1 kb should be enough for short text input
                    //get the numbers of bytes of the input and then save the input to buffer
                    //build a string of the receiving message from the buffer
                    //then print the received message
                    final byte[] buffer = new byte[1024];
                    final int readBytes = inputStream.read(buffer);
                    lsReceivedMessage = new String(buffer, 0, readBytes);
                    if(!lsReceivedMessage.isEmpty())
                    {
                        updateStatus("[SERVER] Message received: " + lsReceivedMessage);
                        loTimestamp = new Timestamp(System.currentTimeMillis());
                    }
                    // Send a message
                    //Send out the message to the other end of the connection
                    updateStatus("[SERVER] Sending message....");
                    outputStream.write(lsMessage.getBytes());
                    outputStream.flush();

                    //Recalculate the session inactive time
                    lnSessionInactiveTime = System.currentTimeMillis() - loTimestamp.getTime();
                }
                updateStatus("[SERVER] Session ended with " + coDevice.getFriendlyName());
                outputStream.close();
                inputStream.close();

            }
            catch(IOException e)
            {
                e.printStackTrace();
            }

        }

    }


    public static void main(String[] args) throws IOException {

        SampleSPPServer sampleSPPServer = new SampleSPPServer();
        sampleSPPServer.run();

    }
}