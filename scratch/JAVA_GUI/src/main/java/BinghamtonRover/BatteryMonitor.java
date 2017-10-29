package BinghamtonRover;

import java.util.Observable;


/**
 * THe BatteryMonitor watches the percentage battery remain on the rover.
 * the battery status will be a number between 0-100.
 */

public class BatteryMonitor extends InformationObserver {

    public BatteryMonitor(){
        super();
    }

    @Override
    public void update(Observable o, Object arg) {
        super.update(o, arg);

        FileUpdatingObservable observable = (FileUpdatingObservable)o;
        System.out.println(
                "The current Battery percentage is: " +
                (long) getJson( observable.getCoFileToMonitor(), "batteryLevel" ) + "%"
        );
    }
}
