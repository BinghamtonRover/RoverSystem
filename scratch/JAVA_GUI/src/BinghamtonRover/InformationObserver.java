package BinghamtonRover;

import java.util.Observable;
import java.util.Observer;

/**
 * This class will be the super() for all gauges/gears on the GUI. From there,
 * we can update them with all the useful information while retaining a sense
 * of order in who/what is listening in on the Observable and maintain proper
 * boundary checking.
 */
public class InformationObserver implements Observer
{
    private static int gnCounter = 0;
    private int cnIDNumber = 0;

    // Establish a unique ID to each instance of this class
    public InformationObserver()
    {
        cnIDNumber = gnCounter++;
    }

    /**
     * When the Observable has a change, this method will be called. The arguments are,
     * for the most part, not useful as we'll be reading the file ourselves.
     * @param o Observable triggering us
     * @param arg Time we were updated at
     */
    @Override
    public void update(Observable o, Object arg)
    {
        System.out.println("InformationObserver [" + cnIDNumber + "] has been updated at " + arg);
    }
}
