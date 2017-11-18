package BinghamtonRover.Video;
import org.bytedeco.javacv.*;
import java.io.IOException;
import java.net.Inet4Address;
import java.net.ServerSocket;
import java.net.Socket;

public class WebCamServer
{
    private static final int gn_port = 5000;

    /**
     * This method creates the video feed obhject and returns it. It does not start it.
     * @return OpenCVFramGrabber the object which will act as a feed for the video
     */
    private static OpenCVFrameGrabber createVideoFeed()
    {
        OpenCVFrameGrabber feed = null;
        try
        {
            //try to grab the main device for the feed, usually the webcam for a laptop for example
            feed = OpenCVFrameGrabber.createDefault(0);
        }
        //if there is no default device then we let the user know.
        catch(FrameGrabber.Exception e)
        {
            e.printStackTrace();
            System.out.println("Couldn't create feed");
            System.exit(0);
        }
        return feed;
    }

    /**
     * This method starts the video feed and displays it Frame by Frame using a CanvasFrame
     * @param feed object which embodies the default device, eg webcam.
     */
    static void startVideoFeed(FrameGrabber feed)
    {
        try
        {
            //start the camera feed
            feed.start();
            Frame frame;
            //create a canvas frame to display the frames coming from the feed
            CanvasFrame cFrame = new CanvasFrame("Server Live Feed", CanvasFrame.getDefaultGamma()/feed.getGamma());
            //while there are frames to grab and while the canvase frame is visible, we update the canvas showing the most current frame grabbed.
            while((frame = feed.grab()) != null){
                if(cFrame.isVisible()){
                    cFrame.showImage(frame);
                }
                //closes the process off when the user closes the video feed
                else System.exit(0);
            }
        }
        //if an exception occurs where we cannot start the feed and display the frames, then we let the user know
        catch(FrameGrabber.Exception e){
            e.printStackTrace();
            System.out.println("Couldn't grab feed");
            System.exit(0);
        }
    }

    /**
     * This method accepts a client that is trying to connect to the server
     * Done with TCP sockets. Need to change this to accomodate UDP protocol.
     * @param srvSocket a ServerSocket which looks to accept client requests.
     * @return clientSocket the socket corresponding to the accepted client.
     * If null is returned, that indicates there was an issue accepting a client.
     */
    private static synchronized Socket acceptClient(ServerSocket srvSocket){
        //first establish connection with client
        Socket clientSocket = null;
        try{
            clientSocket = srvSocket.accept();
            return clientSocket;
        }
        catch(IOException e){
            System.out.println("Server refused to service the client's request");
            System.exit(-1);
        }
        return clientSocket;
    }

    /**
     * This method gets the server up and running, looking for clients that want access to the video feed.
     * A Thread is delegated to service each client, allowing for multiple clients to access the feed concurrently.
     * The server is printing out its port and host inforarmation, which a potential client must type in to try and get access.
     * Ideally, we want to ensure that the server doesn't prematurely die. Right now, a client can cause the server to crash
     * If an error occurs, which is unacceptable.
     * @param args command line arguments that are passed in when WebCamServer is run
     */
    public static void main(String[] args){
        OpenCVFrameGrabber feed = WebCamServer.createVideoFeed();
        try(ServerSocket srvSocket = new ServerSocket(gn_port)){
            Thread feedThread = new Thread(new VideoFeedWorkerRunnable(feed));
            feedThread.start();
            System.out.println("Server's port: " + gn_port);
            System.out.println("Server's host: " + Inet4Address.getLocalHost().getHostAddress());
            //look to accept a client and then send feed to that client
            //in a while loop so multiple clients can potentially connect.
            //also, a client disconnecting does not crash the server and his socket continues to work
            while(true){
                try {
                    Socket clientSocket = acceptClient(srvSocket);
                    System.out.println("Accepted a client!");
                    //start a worker thread to service the client, then look for more clients
                    Thread clientThread = new Thread(new ClientWorkerRunnable(feed, clientSocket));
                    clientThread.start();
                }
                catch(Exception e){
                    System.exit(-1);
                }
            }
        }
        catch(IOException e){
            e.printStackTrace();
            System.out.println("Could not establish a socket for streaming feed");
            System.exit(-1);
        }
    }
}
