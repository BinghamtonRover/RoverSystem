package BinghamtonRover.Bluetooth;

import org.apache.commons.lang3.Validate;

import javax.bluetooth.*;
import javax.microedition.io.Connector;
import javax.microedition.io.StreamConnection;

import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;

import java.util.Vector;

/**
 * A thread that acts as a client
 */
public class SampleSPPClient implements DiscoveryListener {

    //object used for waiting
    private static Object lock=new Object();
    private static int gnClientNumber = 0;

    //vector containing the devices discovered
    private static Vector vecDevices = new Vector();
    private static String connectionURL = null;
    private final LocalDevice coLocalDevice;
    private String csLocalDeviceName;
    private final UUID uuid;
    private DiscoveryAgent coAgent;
    private int cnClientID;

    public SampleSPPClient() {
        gnClientNumber++;
        this.cnClientID = gnClientNumber; //Assign an id to this instance of client
        this.uuid = new UUID("1110", false);

        //use temp variables to hold localDevice and Agent
        LocalDevice loLocalDevice = null;
        DiscoveryAgent loAgent = null;
        //Try to obtain information for local device
        try {
            loLocalDevice = LocalDevice.getLocalDevice();
            updateStatus("[CLIENT"+cnClientID+"] Device Address: " + loLocalDevice.getBluetoothAddress());
            updateStatus("[CLIENT"+cnClientID+"] Device Name: " + loLocalDevice.getFriendlyName());

            loAgent = loLocalDevice.getDiscoveryAgent();
        } catch (BluetoothStateException e) {
            e.printStackTrace();
        }
        //Assign the local temp object to the actual class object

        Validate.notNull(loLocalDevice, "LocalDevice is null");
        Validate.notNull(loAgent, "Agent is null");

        this.coLocalDevice = loLocalDevice;
        this.coAgent = loAgent;
        this.csLocalDeviceName = coLocalDevice.getFriendlyName();
    }


    /**start to search for BT devices in the vicinity
     * Every device found will triggers function call device Discovered
     * and the device will be passed to the method deviceDiscovered
     */
    private void startDeviceInquiry() {
        updateStatus("[CLIENT" + cnClientID + " " + csLocalDeviceName + "] starting device inquiry");
        try {
            coAgent.startInquiry(DiscoveryAgent.GIAC, this);
            //Lock the thread to prevent anything else from happening while inquiring devices
            try {
                synchronized (lock) {
                    lock.wait();
                }
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        } catch (BluetoothStateException e) {
            e.printStackTrace();
        }
    }

        //if this method is called we are expecting to found some devices and print them on the console
        //Prompt the user to select a device to search for service
        //use a bufferReader for input
    private String startServiceInquiry()
    {
        //Prompt the user to choose a device to search service on
        System.out.print("Choose Device index: ");
        BufferedReader bReader = new BufferedReader(new InputStreamReader(System.in));

        String lsUserInput = "0";
        try
        {
            lsUserInput = bReader.readLine();
        } catch (IOException e) {
            e.printStackTrace();
        }
        int lnChosenIndex = Integer.parseInt(lsUserInput.trim());

        //Select the device that the user chosen from the vector of discovered devices
        RemoteDevice remoteDevice=(RemoteDevice)vecDevices.elementAt(lnChosenIndex-1);

        UUID[] uuidSet = new UUID[1];
        uuidSet[0] = uuid;
        int[] attrSet = new int[1];
//        uuidSet[0]=new UUID("1101",false);

        //Start a search for services
        try
        {
            updateStatus("[CLIENT"+cnClientID+"  "+csLocalDeviceName+"] is searching for service on " + remoteDevice.getFriendlyName(true));
            coAgent.searchServices(null,uuidSet,remoteDevice,this);
            //Lock this thread to wait for search to complete
            try {
                synchronized (lock) {
                    lock.wait();
                }
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        catch (BluetoothStateException e)
        {
            e.printStackTrace();
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }
        //once the lock is released, the connectionURL should have something, but need validation
        return connectionURL;

    }

    void updateStatus(String asMessage)
    {
        System.out.println(asMessage);
    }

    //methods of DiscoveryListener
    //Called when DiscoveryListener discovers a new device
    public void deviceDiscovered(RemoteDevice btDevice, DeviceClass cod)
    {
        //add the new device to the vector
        if(!vecDevices.contains(btDevice)){
            vecDevices.addElement(btDevice);
        }
    }

    //upon completion of device inquiry, call this method to unlock the thread
    public void inquiryCompleted(int discType)
    {
        //print all devices in vecDevices
        int deviceCount = vecDevices.size();

        if(deviceCount <= 0){
            System.out.println("No Devices Found.");
            System.exit(0);
        }
        else{
            //print bluetooth device addresses and names in the format [ No. address (name) ]
            System.out.println("BinghamtonRover.Bluetooth Devices: ");
            for (int i = 0; i <deviceCount; i++) {
                RemoteDevice remoteDevice=(RemoteDevice)vecDevices.elementAt(i);
                try
                {
                    System.out.println((i+1)+". "+remoteDevice.getBluetoothAddress()+" ("+remoteDevice.getFriendlyName(true)+")");
                }
                catch (IOException e)
                {
                    e.printStackTrace();
                }
            }
        }

        synchronized(lock)
        {
            lock.notify();
        }

    }

    //implement this method since services are not being discovered
    public void servicesDiscovered(int transID, ServiceRecord[] servRecord)
    {
        updateStatus("[Client"+cnClientID+" "+csLocalDeviceName+"] had discovered a service");
        //Assign the URL of the first service discovered to the connectionURL
        if(servRecord!=null && servRecord.length>0)
        {
            connectionURL=servRecord[0].getConnectionURL(0,false);
        }
    }

    public LocalDevice getCoLocalDevice() {
        Validate.notNull(coLocalDevice, "Local Device is Null");
        return coLocalDevice;
    }

    //called upon completion of service search
    public void serviceSearchCompleted(int transID, int respCode)
    {
        updateStatus("[CLIENT"+cnClientID+"  "+csLocalDeviceName+"] completed searching for service");
        synchronized(lock)
        {
            lock.notify();
        }

        switch (respCode) {

            // If the search is completed and the server URL was found,
            // connect to the URL and handle the connection.
            case DiscoveryListener.SERVICE_SEARCH_COMPLETED:
                if (connectionURL != null) {
                    //start a connection
                    // startConnection(connectionURL);
                }
                break;

            case DiscoveryListener.SERVICE_SEARCH_DEVICE_NOT_REACHABLE:
                updateStatus("[CLIENT"+cnClientID+"  "+csLocalDeviceName+"] Search service device not reachable");
                break;

            case DiscoveryListener.SERVICE_SEARCH_NO_RECORDS:
                updateStatus("[CLIENT"+cnClientID+"  "+csLocalDeviceName+"] Service search has found no records");
                break;

            case DiscoveryListener.SERVICE_SEARCH_ERROR:
                updateStatus("[CLIENT"+cnClientID+"  "+csLocalDeviceName+"] Service search error");
                break;

            case DiscoveryListener.SERVICE_SEARCH_TERMINATED:
                updateStatus("[CLIENT"+cnClientID+"  "+csLocalDeviceName+"] Service search terminated");
                break;
        }


    }


    public static void main(String[] args) throws IOException {


        SampleSPPClient client = new SampleSPPClient();
        client.startDeviceInquiry();
        String serviceURL = client.startServiceInquiry();

        Validate.notNull(serviceURL, "ServiceURL is not found");
        clientServerConnection connection = new clientServerConnection(client, serviceURL);
        connection.run();
    }
}

/**
 * This class is a thread that initiate a connection with the server
 * and will try to send messages to the server
 */
class clientServerConnection extends Thread
    {

        private SampleSPPClient coClient;
        private String connectionURL;
        private LocalDevice coDevice;


        public clientServerConnection(SampleSPPClient aoClient, String asConnectionURL) throws IOException
        {
            this.coClient = aoClient;
            this.connectionURL = asConnectionURL;
            this.coDevice = aoClient.getCoLocalDevice();
        }

        public void run()
        {
            try
            {
                StreamConnection loConnection = null;
                DataOutputStream loOutputStream = null;
                DataInputStream loInputStream = null;


                try
                {
                    // Send the server a request to open a connection
                    loConnection = (StreamConnection) Connector.open(connectionURL);
                    coClient.updateStatus("[CLIENT] SPP session created");

                    //opening data IO stream
                    loInputStream = loConnection.openDataInputStream();
                    loOutputStream = loConnection.openDataOutputStream();
                    BufferedReader loBufReader = new BufferedReader(new InputStreamReader(System.in));

                    //Send the name of this local device before exchanging messages
                    loOutputStream.write(coDevice.getFriendlyName().getBytes());
                    loOutputStream.flush();

                    //Continue to loop until user decide to exit
                    while(true)
                    {
                        // Send a message to the server
                        // Use a for loop to keep the exchange of message going. Didn't really help//                        final String message = "\nJSR-82 CLIENT " + csLocalDeviceName + " says hello!";
                        String lsMessage = loBufReader.readLine();
                        coClient.updateStatus("[CLIENT] Sending message....");

                        loOutputStream.write(lsMessage.getBytes());
                        loOutputStream.flush();


                        String lsReceivedMessage = readFromServer(loOutputStream, loInputStream);
                        coClient.updateStatus("[CLIENT " + coDevice.getFriendlyName() + "] received a Message: " + lsReceivedMessage);
                    }
                }
                finally
                {
                    loOutputStream.close();
                    loInputStream.close();
                    loConnection.close();
                    coClient.updateStatus("[CLIENT " + coDevice.getFriendlyName() + "] SPP session closed");
                }
            }
            catch (IOException ioe)
            {
                ioe.printStackTrace();
            }
        }

        private String readFromServer (DataOutputStream aoDOS, DataInputStream aoDIS)
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

            System.out.println("Nothing was received from the Server");
            return null;
        }



    }


