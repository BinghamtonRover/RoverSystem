package BinghamtonRover.Monitors;

import java.util.Observable;

/*
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
        FileUpdatingObservable loObservable = (FileUpdatingObservable) o;

        String lsLatitude = (String) getJson(loObservable.getCoFileToMonitor(), "latitudeDirection");
        String lsLongitude = (String) getJson(loObservable.getCoFileToMonitor(), "longitudeDirection");

        System.out.println("The target is at " + lsLatitude + lsLongitude + " Direction");
    }
}
