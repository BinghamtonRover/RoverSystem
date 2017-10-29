package BinghamtonRover;

import java.util.Observable;



/**
 * The DistanceMonitor should monitor the distance that the rover have traveled.
 * the Json file provided the total distance the rover travelled
 */

public class DistanceMonitor extends InformationObserver{

    public DistanceMonitor(){
        super();
    }

    @Override
    public void update(Observable o, Object arg) {
        super.update(o, arg);

        FileUpdatingObservable observable = (FileUpdatingObservable)o;
        System.out.println(
                "Total distance travelled is: " +
                (double) getJson( observable.getCoFileToMonitor(), "totalDistanceTraveled" )
        );
    }
}
