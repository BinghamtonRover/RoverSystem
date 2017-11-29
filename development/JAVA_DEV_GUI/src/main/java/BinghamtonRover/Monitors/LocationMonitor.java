package BinghamtonRover.Monitors;

import BinghamtonRover.GuiMain.GuiController;

import java.util.Observable;

/*
 * The location Monitor will read the longitude and latitude from the Json file.
 */

public class LocationMonitor extends InformationObserver
{
    private GuiController coController;

    public LocationMonitor()
    {
        super();
    }

    public LocationMonitor(GuiController loController)
    {
        super();

        coController=loController;
    }

    @Override
    public void update(Observable o, Object arg)
    {
        FileUpdatingObservable loObservable = (FileUpdatingObservable) o;

        double lfLatitude = (double) getJson(loObservable.getCoFileToMonitor(), "latitude");
        double lfLongitude = (double) getJson(loObservable.getCoFileToMonitor(), "longitude");

        if (coController != null) coController.updateLatitude("The current location is: " + lfLatitude + ", " + lfLongitude );
    }
}
