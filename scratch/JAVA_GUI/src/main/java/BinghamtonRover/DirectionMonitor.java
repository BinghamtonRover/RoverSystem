package BinghamtonRover;

import java.util.Observable;


/**
 * Assume this class monitors the target's relative direction to the rover.
 */

public class DirectionMonitor extends InformationObserver
{

    public DirectionMonitor()
    {
        super();
    }

    @Override
    public void update(Observable o, Object arg)
    {
        //Call super method to acknowledge user that this Observer has been updated
        super.update(o, arg);

        //Print out the information that this observer is monitoring
        FileUpdatingObservable loObservable = (FileUpdatingObservable)o;
        System.out.println
        (
                "The target is at " +
                getJson( loObservable.getCoFileToMonitor(), "latitudeDirection" ) +
                getJson( loObservable.getCoFileToMonitor(), "longitudeDirection") +
                " Direction"
        );
    }
}
