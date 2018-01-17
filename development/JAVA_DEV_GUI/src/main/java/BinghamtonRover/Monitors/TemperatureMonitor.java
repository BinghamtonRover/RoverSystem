package BinghamtonRover.Monitors;

import BinghamtonRover.GuiMain.GuiController;

import java.util.Observable;

/*
 * The Temperature Monitor should monitor the temperature around the rover.
 * The New json file shows that the temperature will probably be in Fahrenheit.
 */

public class TemperatureMonitor extends InformationObserver
{
    private GuiController coController;

    public TemperatureMonitor()
    {
        super();
    }

    public TemperatureMonitor(GuiController loController)
    {
        super();

        coController=loController;
    }

    @Override
    public void update(Observable o, Object arg)
    {
        FileUpdatingObservable loObservable = (FileUpdatingObservable) o;

        double lfTemperature = (double) getJson(loObservable.getCoFileToMonitor(), "temperature");

        if (coController != null) coController.updateTemperature("The current temperature is: " + lfTemperature + "°F" );
    }
}
