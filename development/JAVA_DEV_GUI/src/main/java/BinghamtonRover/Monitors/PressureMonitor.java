package BinghamtonRover.Monitors;

import java.util.Observable;

/*
 * Pressure monitor monitors the air pressure.
 */

public class PressureMonitor extends InformationObserver
{
    public PressureMonitor()
    {
        super();
    }

    @Override
    public void update(Observable o, Object arg)
    {
        FileUpdatingObservable loObservable = (FileUpdatingObservable) o;

        double lfPressure = (double) getJson(loObservable.getCoFileToMonitor(), "pressure");

        System.out.println("The air pressure is: " + lfPressure + "stp");
    }
}
