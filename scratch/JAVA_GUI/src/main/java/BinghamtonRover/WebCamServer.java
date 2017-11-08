package BinghamtonRover;
import org.bytedeco.javacv.*;
import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketAddress;

public class WebCamServer {
    private static final int port = 5000;
    private static OpenCVFrameGrabber createVideoFeed(){
        OpenCVFrameGrabber feed = null;
        try {
            feed = OpenCVFrameGrabber.createDefault(0);
        }
        catch(FrameGrabber.Exception e){
            e.printStackTrace();
            System.out.println("Couldn't create feed");
            System.exit(0);
        }
        return feed;
    }

    static void startVideoFeed(FrameGrabber feed) {
        try{
            feed.start();
            Frame frame;
            CanvasFrame cFrame = new CanvasFrame("Server Live Feed", CanvasFrame.getDefaultGamma()/feed.getGamma());
            while((frame = feed.grab()) != null){
                if(cFrame.isVisible()){
                    cFrame.showImage(frame);
                }
                //closes the process off when the user closes the video feed
                else System.exit(0);
            }
        }
        catch(FrameGrabber.Exception e){
            e.printStackTrace();
            System.out.println("Couldn't grab feed");
            System.exit(0);
        }
    }

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

    public static void main(String[] args){
        OpenCVFrameGrabber feed = WebCamServer.createVideoFeed();
        try(ServerSocket srvSocket = new ServerSocket(port)){
            Thread feedThread = new Thread(new VideoFeedWorkerRunnable(feed));
            feedThread.start();
            SocketAddress endpoint = srvSocket.getLocalSocketAddress();
            System.out.println("Server's port: " + port);
            System.out.println("Server's host: " + endpoint);
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
                        System.exit(-2);
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
