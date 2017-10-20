package ObserverDemo;

import java.io.File;

public class AutoFileRead implements Runnable{

    private File file;
    private Stock stock;

    public AutoFileRead(Stock stock, String url){
        this.file = new File(url);
        this.stock = stock;
    }

    @Override
    public void run() {

        long fileLastMod = file.lastModified();

        while(true){
            try {
                Thread.sleep(1000);
            }
            catch (InterruptedException e){
                System.out.println("ERROR: Thread Interrupted");
            }


            if (fileLastMod != file.lastModified()){
                stock.notifyObservers( file.lastModified() );
                fileLastMod = file.lastModified();
            }


        }
    }
}
