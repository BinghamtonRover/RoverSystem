import java.util.*;
import java.nio.file.*;
import java.nio.file.attribute.*;

class Servable extends Observable {
    public long time;
    
    public Servable(long Time) {
        time = Time;
    } 
    
    public void tick() {
        if (time >= 0) time--;
        
        if (time==0) {
            setChanged();
            notifyObservers();
        }
    }
    
    public void printFileData(String file_path) {
        Path file = Paths.get(file_path);
        try {
        BasicFileAttributes attr = Files.readAttributes(file, BasicFileAttributes.class);
        String owner = Files.getOwner(file).getName();
        } catch (IOException e) {}

        System.out.println("creationTime: " + attr.creationTime());
        System.out.println("lastAccessTime: " + attr.lastAccessTime());
        System.out.println("lastModifiedTime: " + attr.lastModifiedTime());
        System.out.println("owner: " + owner);
        
    }
    
    public static void main(String[] args) {
        Servable served = new Servable(2000000000);
        
        served.printFileData("server.java");
    }
}
