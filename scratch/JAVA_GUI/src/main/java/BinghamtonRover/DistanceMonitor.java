package BinghamtonRover;

import java.util.Observable;



/**
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
        //Call super method to acknowledge user that this Observer has been updated
        super.update(o, arg);

        //Print out the information that this observer is monitoring
        FileUpdatingObservable loObservable = (FileUpdatingObservable)o;
        System.out.println
        (
                "Total distance travelled is: " +
                (double) getJson( loObservable.getCoFileToMonitor(), "totalDistanceTraveled" )
        );
    }
}
