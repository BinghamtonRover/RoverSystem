package BinghamtonRover;

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

    private Client(String h, int p){
        host = h;
        port = p;
    }

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
