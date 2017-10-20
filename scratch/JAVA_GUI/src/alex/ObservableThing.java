
/**
 * Write a description of class ObservableThing here.
 *
 * @author (your name)
 * @version (a version number or a date)
 */
import java.util.ArrayList;
import java.util.List;
import java.util.Observable;
public class ObservableThing extends Observable
{
    // instance variables - replace the example below with your own
    private ArrayList <ObserverDude> observerList = new ArrayList<ObserverDude>();
    private double lastUpdated;
    /**
     * Constructor for objects of class ObservableThing
     */
    public ObservableThing(double lastUpdated)
    {
        // initialise instance variables
        this.lastUpdated = lastUpdated;
    }

    public void changeValue(double changeTo)
    {
        // put your code here
        this.lastUpdated = changeTo;
        setChanged();
        notifyObservers();
    }
    
    /*public void notifyObservers()
    {
        for (int i = 0; i<observerList.size(); i++)
        {
            observerList.get(i).update();
        }
    }*/
    
}
