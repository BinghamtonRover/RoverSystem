package BinghamtonRover.Video;

import org.bytedeco.javacpp.opencv_core;
import org.bytedeco.javacv.Frame;
import org.bytedeco.javacv.OpenCVFrameConverter;
import org.bytedeco.javacv.OpenCVFrameGrabber;

import java.io.*;
import java.net.Socket;
import java.net.SocketException;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

import static org.bytedeco.javacpp.opencv_imgcodecs.cvSaveImage;

public class ClientWorkerRunnable implements Runnable{
    private OpenCVFrameGrabber feed;
    private Socket clientSocket;

    ClientWorkerRunnable(OpenCVFrameGrabber f, Socket c){
        feed = f;
        clientSocket = c;
    }

    /**
     * This method is still in the works. We are currently trying to figure out how to turn a frame
     * into a byte array, which should be serializable. We will then send the byte array over to the
     * client, who will then take the btyes, reassemble the frame, and display the frames one at a time.
     * Not sure if this is the best way to go about this, but that is the idea we had in mind.
     */
    @Override
    public void run() {
        AtomicReference<ObjectOutputStream> out = new AtomicReference<>();
        opencv_core.IplImage img;
        try {
            out.set(new ObjectOutputStream(clientSocket.getOutputStream()));
            Frame frame;
            while((frame = feed.grab()) != null) {
                //OpenCVFrameConverter frameToImg = new OpenCVFrameConverter.ToIplImage();
                //img = (opencv_core.IplImage) frameToImg.convert(frame);
                //cvSaveImage("frame.jpg", img);
                //FileInputStream fis=new FileInputStream("frame.jpg");
//                AtomicInteger x = new AtomicInteger();
//                while(true){
//                    x.set(fis.read());
//                    if(x.get() ==-1)break;
//                    out.get().write(x.get());
//                }
                out.get().writeObject(frame);
            }
            out.get().close();
        }
        catch(SocketException e){
            System.out.println("Client disconnected.");
        }
        catch (IOException e) {
            e.printStackTrace();
        }
    }
}
