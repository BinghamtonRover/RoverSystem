# Branch Summary

We took the branch from Bluetooth Demo and went through it and
made it work. Now able to establish a connection between two pc via bluetooth
and exchange strings one at a time (If you scrambled the sequence the program 
will glitch out). We kinda started on the GUI but could not finish it. 
We have an idea for multi-client connection but am not sure.

The blueCove 2.1.1 library imported by maven dosen't seems to work. 
It needs to be externally downloaded and imported.

Download link: http://www.java2s.com/Code/Jar/b/Downloadbluecove211jar.htm

Also there is a unknown issue of using this library on Mac.

The Program run like this: <br>
* Run the BluetoothGUI 
* When the start Server Button is pressed, the SampleSSPServer, the server starts and waits for client to request connection
* Run the SampleSSPClient, the client will first search for near by devices, 
    then print a list of detected devices and prompt the user to select a device
* After the device is selected, the client will search for service on that selected device
* Once the service search is complete, if the client discovered a service, it will form a connection
    between the server and client.
* The Server will initiate a new thread which handles the communication between itself and the client.

Notes:
* One known Bug is that your cannot shutdown the server and then start it again.
* The GUI for Client is not fully implemented, but the client bluetooth work, use the System.out for now for testing
* The MultiClient functionality was unable to be fully tested due to the limited numbers of bluetooth enabled computer