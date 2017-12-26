# Branch Summary

We took the branch from BinghamtonRover.Bluetooth Demo and went through it and
made it work. Now able to establish a connection between two pc via bluetooth
and exchange strings one at a time (If you scrambled the sequence the program 
will glitch out). We kinda started on the GUI but could not finish it. 
We have an idea for multi-client connection but am not sure.

The blueCove 2.1.1 library imported by maven dosen't seems to work. 
It needs to be externally downloaded and imported.

Download link: http://www.java2s.com/Code/Jar/b/Downloadbluecove211jar.htm

Also there is a unknown issue of using this library on Mac.

The Program run like this: <br>
* Run the SampleSSPServer, the server starts and waits for client to request connection
* Run the SampleSSPClient, the client start, it will first search for near by devices, 
    then print a list of detected devices and prompt the user to select a device
* After the device is selected, the client will search for service on that selected device
* Once the search is complete, if the client discovered a service, it will spawn
    a new thread which handles the messaging between itself and the server.
* Currently the Thread will be running continuously with a while(true) so that the 
    exchange of message will not be terminated.