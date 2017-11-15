package BinghamtonRover.Bluetooth;

import javafx.fxml.FXML;

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


/**
 * A thread that opens an SPP connection, awaits client requests, accepts
 * incoming messages, and sends a response.
 */
public class SampleSPPServer implements Runnable
{
    //Reference of the component of the GUI

    //GUI Component not yet implemented
    //Not sure if we want to use this class as the controller
//    @FXML private Button coSendMessageButton;
//    @FXML private TextArea coMessageBoard;
//    @FXML private TextField coMessageInput;

    private static final String gsSERVICE_NAME_SSP = "The SSP Server";


    public void run()
    {
        try
        {
            StreamConnection connection;
            DataOutputStream outputStream = null;
            DataInputStream inputStream = null;
            //Make this bufferReader final so it can be used in lambda
            final BufferedReader loBufReader = new BufferedReader(new InputStreamReader(System.in));

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

                //Opening IOStream and system input reader
                inputStream = connection.openDataInputStream();
                outputStream = connection.openDataOutputStream();
                while(true) {

                    String lsMessage, lsReceivedMessage = null;
//
                    // Read a message
                    //a buffer byte array used to temporarily hold the input, 1 kb should be enough for short text input
                    final byte[] buffer = new byte[1024];
                    //get the numbers of bytes of the input and then save the input to buffer
                    final int readBytes = inputStream.read(buffer);
                    //build a string of the receiving message from the buffer
                    lsReceivedMessage= new String(buffer, 0, readBytes);
                    //Print the received message
                    updateStatus("[SERVER] Message received: " + lsReceivedMessage);

                    // Send a message
                    lsMessage = loBufReader.readLine();
                    updateStatus("[SERVER] Sending message....");
                    //Send out the message to the other end of the connection
                    outputStream.write(lsMessage.getBytes());
                    outputStream.flush();

                }
            }

            finally
            {
                outputStream.close();
                inputStream.close();
                loBufReader.close();
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
//        coMessageBoard.appendText(asMessage);
    }

    /**
     * Here is a thread class that will be created every time a connection is made
     * This thread is expected to handle the communication between the client and server
     *
     * when the server received a connection request, it will establish a connection and
     * instantiate a SeverClientConnection thread that listens to the message the client
     * sent and respond with, for now, a response string
     * Not yet implemented
     */
    class ServerClientConnection extends Thread{

    }



    public static void main(String[] args) throws IOException {



        //start the Server
        SampleSPPServer sampleSPPServer = new SampleSPPServer();
        sampleSPPServer.run();
//
//        //Launch the GUI
//        launch(args);
    }

    //not sure where to place the GUI controller
//    @Override
//    public void start(Stage aoPrimaryStage) throws Exception
//    {
//        Parent loRoot = FXMLLoader.load(getClass().getResource("/fxml/BluetoothGUI.fxml"));
//        aoPrimaryStage.setTitle("BluetoothGUI");
//        aoPrimaryStage.setScene(new Scene(loRoot));
//        aoPrimaryStage.show();
//    }

}