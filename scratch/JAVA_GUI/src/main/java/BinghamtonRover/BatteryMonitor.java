package BinghamtonRover;

import java.util.Observable;


/**
 * THe BatteryMonitor watches the percentage battery remain on the rover.
 * the battery status will be a number between 0-100.
 */

public class BatteryMonitor extends InformationObserver
{

    public BatteryMonitor()
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
                "The current Battery percentage is: " +
                getJson( loObservable.getCoFileToMonitor(), "batteryLevel" ) + "%"
        );
    }
}
