package BinghamtonRover;

import java.util.Observable;


/**
 * Assume this class monitors the target's relative direction to the rover.
 */

public class DirectionMonitor extends InformationObserver
{

    public DirectionMonitor(){
        super();
    }

    @Override
    public void update(Observable o, Object arg)
    {
        super.update(o, arg);

        FileUpdatingObservable observerable = (FileUpdatingObservable)o;
        System.out.println(
                "The target is at " +
                (String) getJson( observerable.getCoFileToMonitor(), "latitudeDirection" ) +
                (String) getJson( observerable.getCoFileToMonitor(), "longitudeDirection") +
                " Direction"
        );
    }
}
