package BinghamtonRover;

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
