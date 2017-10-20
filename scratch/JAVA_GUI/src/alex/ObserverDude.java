
/**
 * Write a description of class Observer here.
 *
 * @author (your name)
 * @version (a version number or a date)
 */
import java.util.Observer;
import java.util.Observable;
public class ObserverDude implements Observer 
{
    // instance variables - replace the example below with your own
    private ObservableThing thing = null;

    /**
     * Constructor for objects of class Observer
     */
    public ObserverDude(ObservableThing thing)
    {
        // initialise instance variables
        this.thing = thing;
    }

    
    public void update(Observable obs, Object obj)
    {
      if (obs == thing)
      {
        System.out.println("Got it1");
      }
      System.out.println("Got it2");
      
    }
}
