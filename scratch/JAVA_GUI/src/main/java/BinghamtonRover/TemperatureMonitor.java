package BinghamtonRover;

import java.util.Observable;


/**
 * The Temperature Monitor should monitor the temperature around the rover.
 * The New json file shows that the temperature will probably be in Fahrenheit.
 */

public class TemperatureMonitor extends InformationObserver
{

    public TemperatureMonitor(){
        super();
    }

    @Override
    public void update(Observable o, Object arg)
    {
        super.update(o, arg);

        FileUpdatingObservable observable = (FileUpdatingObservable)o;
        System.out.println(
                "The current temperature is: " +
                        (double) getJson( observable.getCoFileToMonitor(), "temperature" ) + " F"
        );
    }
}
