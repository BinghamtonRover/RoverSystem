package BinghamtonRover.Video;

import org.bytedeco.javacpp.opencv_core;
import org.bytedeco.javacv.*;
import java.io.*;
import java.net.ConnectException;
import java.net.Socket;
import java.util.Scanner;

import static org.bytedeco.javacpp.opencv_imgcodecs.cvLoadImage;

public class Client {
    private String host;
    private int port;

    /**
     * Client value constructor
     * @param h the host
     * @param p the port number
     */
    private Client(String h, int p){
        host = h;
        port = p;
    }

    /**
     * This method is meant to connect a client to the Server.
     * @return clientSocket the socket corresponding to the Client's connection to the Server
     */
    private Socket connectClient(){
        Socket clientSocket = null;
        try {
            //this call automatically calls connect for us.
            //so we do not need to call connect on it.
            clientSocket = new Socket(host, port);
        }
        catch(ConnectException e){
            System.out.println("Could not establish a socket for streaming feed. Exiting.");
            System.exit(-1);
        }
        catch (IOException e) {
            e.printStackTrace();
        }
        return clientSocket;

    }

    /**
     * This method displays a canvasFrame for the client, and tries to display each frame
     * sent by the server in real time. We are still working out how to serialize the
     * frames for successful transfer. The original idea of converting a frame to an image sort of worked,
     * but the byte array option will probably prove more efficient and better for playing with the frame data later on if needed.
     * @param clientSocket The socket corresponding to the client's connection with the Server
     */
    private static synchronized void displayFeed(Socket clientSocket){
        CanvasFrame canvas = new CanvasFrame("Client Webcam feed");
        try {
            while (clientSocket.isConnected()) {
                Thread extractThread = new Thread(new ExtractFrameRunnable(clientSocket.getInputStream(), new FileOutputStream("sentFrame.jpg")));
                extractThread.start();
                extractThread.join();
                opencv_core.IplImage img = cvLoadImage("sentFrame.jpg");
                OpenCVFrameConverter imgToFrame = new OpenCVFrameConverter.ToIplImage();
                canvas.showImage(imgToFrame.convert(img));
            }
        }
        catch(NullPointerException e){
            e.getStackTrace();
        }
        catch(IOException e) {
            e.getStackTrace();
            System.out.println("Lost connection to server. Exiting now.");
            System.exit(-1);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    /**
     * This method simply scans for the server's host and port number, and then
     * the client creates a socket to try and establish a connect with the server.
     * This needs to be changed from TCP to UDP protocol, so that it's connectionless.
     * @param args command line arguments that are included when the program is run at the command line
     */
    public static void main(String[] args) {
        Scanner reader = new Scanner(System.in);
        System.out.println("Enter the server's host address: ");
        String host = reader.next();
        System.out.println("Enter the server's port: ");
        int port = reader.nextInt();
        Client client = new Client(host, port);
        Socket clientSocket = client.connectClient();
        Client.displayFeed(clientSocket);

    }
}
