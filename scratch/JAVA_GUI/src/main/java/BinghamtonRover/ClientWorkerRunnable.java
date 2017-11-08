package BinghamtonRover;

import org.bytedeco.javacpp.opencv_core;
import org.bytedeco.javacv.Frame;
import org.bytedeco.javacv.OpenCVFrameConverter;
import org.bytedeco.javacv.OpenCVFrameGrabber;

import java.io.FileInputStream;
import java.io.IOException;
import java.io.OutputStream;
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

    @Override
    public void run() {
        AtomicReference<OutputStream> out = new AtomicReference<>();
        opencv_core.IplImage img;
        try {
            out.set(clientSocket.getOutputStream());
            Frame frame;
            while((frame = feed.grab()) != null) {
                OpenCVFrameConverter frameToImg = new OpenCVFrameConverter.ToIplImage();
                img = (opencv_core.IplImage) frameToImg.convert(frame);
                cvSaveImage("frame.jpg", img);
                FileInputStream fis=new FileInputStream("frame.jpg");
                AtomicInteger x = new AtomicInteger();
                while(true){
                    x.set(fis.read());
                    if(x.get() ==-1)break;
                    out.get().write(x.get());
                }
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
