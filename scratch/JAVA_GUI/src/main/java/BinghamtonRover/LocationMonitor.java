package BinghamtonRover;

import java.util.Observable;

/*
 * The location Monitor will read the longitude and latitude from the Json file.
 */

public class LocationMonitor extends InformationObserver
{
    public LocationMonitor()
    {
        super();
    }

    @Override
    public void update(Observable o, Object arg)
    {
        FileUpdatingObservable loObservable = (FileUpdatingObservable) o;

        double lfLatitude = (double) getJson(loObservable.getCoFileToMonitor(), "latitude");
        double lfLongitude = (double) getJson(loObservable.getCoFileToMonitor(), "longitude");

        System.out.println("The current location is: " + lfLatitude + ", " + lfLongitude);
    }
}
