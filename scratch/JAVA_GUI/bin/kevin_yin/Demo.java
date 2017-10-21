import java.util.Scanner;
import cabbagepkg.*;
import java.io.File;

public class Demo {

    public static void main(String[] args){
        File file = new File("readme.md"); //this is the file where we will be checking the lastModified date 
        CabbageStock stock = new CabbageStock();
        stock.addObserver(new Customer("Kevin")); //add an observer
        //Lambda Expression Thread lol
        Thread thread = new Thread(() -> {
            try{
                long lastModified_time = file.lastModified(), new_time;
                while(true){
                    //checks if the current lastModified time of the readme file is greater than the past lastModified time
                    if(file.lastModified() > lastModified_time){ 
                        lastModified_time = file.lastModified();
                        stock.setStock(); //calls the setStock method in the CabbageStock (Observable) Class
                    }
                    Thread.sleep(1000); //delay by 1 second
                }
            } catch(Exception e){
                System.out.println(e);
            }
        });
        thread.start(); //start thread
    }
}