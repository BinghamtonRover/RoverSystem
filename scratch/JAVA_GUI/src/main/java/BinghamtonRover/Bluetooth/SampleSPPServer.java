package BinghamtonRover.Bluetooth;

import org.apache.commons.lang3.Validate;

import javax.bluetooth.BluetoothStateException;
import javax.bluetooth.LocalDevice;
import javax.bluetooth.UUID;
import javax.microedition.io.Connector;
import javax.microedition.io.StreamConnection;
import javax.microedition.io.StreamConnectionNotifier;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;

import java.util.concurrent.Executor;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;


/**
 * A thread that opens an SPP connection, awaits client requests, accepts
 * incoming messages, and sends a response.
 */
public class SampleSPPServer implements Runnable {

    private BluetoothServerGUIController coController;
    private static final String gsSERVICE_NAME_SSP = "The SSP Server";
    private boolean cbServerOpen = false;

    public SampleSPPServer(BluetoothServerGUIController aoController)
    {
        coController = aoController;
    }

    public void run() {
        OpenServer();

            try {
                //Generate a new UUID and get the local device information
                final UUID uuid = new UUID("1110", false);
                final LocalDevice local = LocalDevice.getLocalDevice();

                //use the updateStatus method to print to console for now
                //Later will use updateStatus to update the GUI
                updateStatus("[SERVER] Device Address: " + local.getBluetoothAddress());
                updateStatus("[SERVER] Device Name: " + local.getFriendlyName());
                updateStatus("[SERVER] Listening for Client...");

                final StreamConnectionNotifier service = (StreamConnectionNotifier) Connector.open("btspp://localhost:" + uuid + ";name=" + gsSERVICE_NAME_SSP);

                while(cbServerOpen)
                {
                    // Open a connection and wait for client requests
                    // spawn a thread for each client connection in order to listen to multiple client simultaneously
                    StreamConnection loConnection = service.acceptAndOpen();

                    //Create a new thread to handle each session
                    ServerClientConnection loSCC = new ServerClientConnection(loConnection);

                    //Use Executor to execute the thread
                    ExecutorService loConnectionBuilder = Executors.newCachedThreadPool();
                    loConnectionBuilder.execute(loSCC);
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
        Validate.notNull(coController,"updateStatus trying to access null Controller");
        coController.updateText(asMessage);
    }

    /**
     * This is a thread class that will be created every time a connection is made
     * This thread is expected to handle the communication between the client and server.
     * when the server received a connection request, it will establish a connection and
     * instantiate a SeverClientConnection thread that listens to the message the client
     * sent and respond with a response string.
     */
    class ServerClientConnection extends Thread {

        private StreamConnection coConnection;
        private String csClientName = "Being X";

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


                //Get the name of the client that is connected
                //Upon the establishment of connection the client is
                //expected to send their name as String via dataInput
                csClientName = readFromClient(outputStream, inputStream);

                updateStatus("[SERVER] SPP session created with " + csClientName);
                updateStatus("[SERVER] Session started at: " + System.currentTimeMillis());

                //this should always evaluate to true but the IDE won't treat this as an infinite loop
                while(cbServerOpen)
                {
                    String lsMessage =  "Server Says: Hello " + csClientName;
                    String lsReceivedMessage = readFromClient(outputStream, inputStream);

                    updateStatus("[SERVER] Received message from " + csClientName +": " + lsReceivedMessage);

                    // Send a message
                    //Send out the message to the other end of the connection
                    updateStatus("[SERVER] Sending message to " + csClientName + "...");
                    outputStream.write(lsMessage.getBytes());
                    outputStream.flush();

                }

                outputStream.close();
                inputStream.close();
                coConnection.close();
            }
            catch(IOException e)
            {
                e.printStackTrace();
            }

        }

        private String readFromClient (DataOutputStream aoDOS, DataInputStream aoDIS)
        {
            Validate.notNull(aoDOS, "DataOutputStream is null");
            Validate.notNull(aoDIS, "DataInputStream is null");

            try {
                // Reading a message
                // Create a buffer byte array used to temporarily hold the input, 1 kb should be enough for short input.
                // get the numbers of bytes of the input and then save the input to buffer
                // build a string of the receiving message from the buffer
                // then return the received message
                final byte[] buffer = new byte[1024];
                final int readBytes = aoDIS.read(buffer);
                String lsReceivedMessage = new String(buffer, 0, readBytes);
                if (!lsReceivedMessage.isEmpty())
                {
                    return lsReceivedMessage;
                }
            }
            catch (IOException e)
            {
                e.printStackTrace();
            }

            System.out.println("Nothing was received from " + csClientName);
            return null;
        }

    }

    public void CloseServer()
    {
        cbServerOpen = false;
        updateStatus("[SERVER] Is trying to Close");
    }

    public void OpenServer()
    {
        cbServerOpen = true;
        updateStatus("[SERVER] Is Opened");
    }


}