package BinghamtonRover.Video;

import java.io.*;
import java.net.Socket;

public class ExtractFrameRunnable implements Runnable{
   /**
     * This method is still in the works. It is meant to be called by a client.
     * The thread will be in charge of extracting byte arrays sent over by a server,
     * and then reassembling those btyes to a frame. We probably should just have this run method also
     * do the work of displaying the frames once assembled.
     */

    private Socket clientSocket;

    public ExtractFrameRunnable(Socket cSock){
        clientSocket = cSock;
    }

    @Override
    public void run() {
        //create a jpg sentFrame that will receive the frame data sent by the server.

        File file = new File("sentFrame.jpg");
        try (OutputStream os = new BufferedOutputStream(new FileOutputStream(file));
             DataInputStream dis = new DataInputStream(new DataInputStream(clientSocket.getInputStream()))
        ) {
            long arrlen = dis.readLong();
            for (int i = 0; i < arrlen; i++) {
                os.write(dis.read());
            }
            os.flush();
            os.close();
        } catch (EOFException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.getStackTrace();
            System.out.println("Lost connection to server. Exiting now.");
            System.exit(-1);
        }
    }
}
