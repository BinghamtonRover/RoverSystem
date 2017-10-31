package BinghamtonRover;

import java.util.Observable;


/**
 * The Temperature Monitor should monitor the temperature around the rover.
 * The New json file shows that the temperature will probably be in Fahrenheit.
 */

public class TemperatureMonitor extends InformationObserver
{

    public TemperatureMonitor()
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
                "The current temperature is: " +
                        (double) getJson( loObservable.getCoFileToMonitor(), "temperature" ) + " F"
        );
    }
}
