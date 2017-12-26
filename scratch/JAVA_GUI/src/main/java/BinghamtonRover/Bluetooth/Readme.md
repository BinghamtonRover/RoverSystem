# Branch Summary


The blueCove 2.1.1 jar imported by maven dosen't seems to work. 
It needs to be externally downloaded and imported.

Download link: http://snapshot.bluecove.org/distribution/download/2.1.1-SNAPSHOT/2.1.1-SNAPSHOT.62/
For Linux, also download and import the glp version along side the regular jar file, its on the same site

Also there is a unknown issue of using Bluecove library on Mac.

The Program run like this: <br>
* Run the BluetoothServerGUI, a simple GUI show up with a text area and a button
* When the start Server Button is pressed, the SampleSSPServer starts and waits for client to request connection
* Run the SampleSSPClient, the client will first search for near by devices, 
    then print a list of detected devices and prompt the user to select a device
* After the device is selected, the bluetooth clie1nt will search for service on that selected device
* Once the service search is complete, if the client discovered a service, it will form a connection
    between the server and client.
* The Server will initiate a new thread which handles the communication between itself and the client.

Notes:
* The GUI for Client is not fully implemented, but the client bluetooth work, use the System.out for now for testing
* The MultiClient functionality was unable to be fully tested due to the limited numbers of bluetooth enabled computer
* If run into problem with SDP Server, try this solution: https://stackoverflow.com/questions/30946821/bluecove-with-bluez-chucks-can-not-open-sdp-session-2-no-such-file-or-direct