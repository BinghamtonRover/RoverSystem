package BinghamtonRover.Video;

import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.net.SocketException;

public class ExtractFrameRunnable implements Runnable{
    private InputStream in;
    FileOutputStream file;

    ExtractFrameRunnable(InputStream i, FileOutputStream f){
        if(i != null && f != null){
            in = i;
            file = f;
        }
    }

    /**
     * This method is still in the works. It is meant to be called by a client.
     * The thread will be in charge of extracting byte arrays sent over by a server,
     * and then reassembling those btyes to a frame. We probably should just have this run method also
     * do the work of displaying the frames once assembled.
     */
    @Override
    public void run(){
        //create a jpg sentFrame that will receive the frame data sent by the server.
        int x;
        while(true){
            try{
                if(in.available() != 0){
                    x = in.read();
                }
                else x = -1;
                if(x==-1){
                    break;
                }
                file.write(x);
            }
            catch(SocketException e){
                e.printStackTrace();
            }
            catch(IOException e){
                e.printStackTrace();
            }
        }
        try {
            file.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
