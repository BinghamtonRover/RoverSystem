package BinghamtonRover.Bluetooth;

import javafx.application.Application;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Parent;
import javafx.scene.Scene;
import javafx.stage.Stage;
import org.apache.commons.lang3.Validate;

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

    static FXMLLoader coLoader = new FXMLLoader();


    private static final String gsSERVICE_NAME_SSP = "The SSP Server";

    //This reference to the GUIController will be called when we update the system message
//    private BluetoothGUIController coGUIController;
//
//    public SampleSPPServer(BluetoothGUIController aoController)
//    {
//        Validate.notNull(aoController,"SPPServer Constructor receieved null controller");
//        coGUIController = aoController;
//    }
//
//    public SampleSPPServer()
//    {
//        coGUIController = new BluetoothGUIController();
//    }


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
//        Validate.notNull(coGUIController,"updateStatus trying to access null Controller");
//        coGUIController.updateText(asMessage);
    }

    /**
     * Here is a thread class that will be created every time a connection is made
     * This thread is expected to handle the communication between the client and server.
     * when the server received a connection request, it will establish a connection and
     * instantiate a SeverClientConnection thread that listens to the message the client
     * sent and respond with a response string
     * Not yet implemented
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

                //Instantiate a timestamp to keep time of connection session time.
                Timestamp loTimestamp = new Timestamp(System.currentTimeMillis());
                updateStatus("Session started at: " + loTimestamp);


                long lnSessionInactiveTime = System.currentTimeMillis() - loTimestamp.getTime();

                //Get the name of the client that is connected
                //Upon the establishment of connection the client is
                //expected to send their name as String via dataInput
                csClientName = readFromClient(outputStream, inputStream);

                //this should always evaluate to true but the IDE won't treat this as an infinite loop
                while(lnSessionInactiveTime < 10000)
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


    public static void main(String[] args) throws IOException {


//       SampleSPPServer sampleSPPServer = new SampleSPPServer(null);
    }
}