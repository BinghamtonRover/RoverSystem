package BinghamtonRover.Monitors;

import java.util.Observable;

/*
 * The DistanceMonitor should monitor the distance that the rover have traveled.
 * the Json file provided the total distance the rover travelled
 */

public class DistanceMonitor extends InformationObserver
{
    public DistanceMonitor(){
        super();
    }

    @Override
    public void update(Observable o, Object arg)
    {
        FileUpdatingObservable loObservable = (FileUpdatingObservable) o;

        double lfDistance = (double) getJson(loObservable.getCoFileToMonitor(), "totalDistanceTraveled");

        System.out.println("Total distance traveled is: " + lfDistance);
    }
}
