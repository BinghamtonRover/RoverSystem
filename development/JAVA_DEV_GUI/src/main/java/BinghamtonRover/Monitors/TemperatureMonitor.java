package BinghamtonRover.Monitors;

import java.util.Observable;

/*
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
        FileUpdatingObservable loObservable = (FileUpdatingObservable) o;

        double lfTemperature = (double) getJson(loObservable.getCoFileToMonitor(), "temperature");

        System.out.println("The current temperature is: " + lfTemperature + "°F");
    }
}
