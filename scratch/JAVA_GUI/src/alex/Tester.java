
/**
 * Write a description of class Tester here.
 *
 * @author (your name)
 * @version (a version number or a date)
 */
public class Tester
{
    public static void main(String[] args) {
        
        ObservableThing fileThing = new ObservableThing(12);
        
        ObserverDude dude = new ObserverDude(fileThing);
        
        fileThing.addObserver(dude);
        
        System.out.println("First change: 15");   
        fileThing.changeValue(15);
        System.out.println("Second change: 32");    
        fileThing.changeValue(32);
        
        
        
    }
}
