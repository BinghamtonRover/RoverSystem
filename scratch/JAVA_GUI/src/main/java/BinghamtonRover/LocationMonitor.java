package BinghamtonRover;


import java.util.Observable;


/**
 * The location Monitor will read the longitude and latitude from the Json file.
 */

public class LocationMonitor extends InformationObserver
{

    public LocationMonitor(){
        super();
    }

    @Override
    public void update(Observable o, Object arg)
    {
        super.update(o, arg);

        FileUpdatingObservable observable = (FileUpdatingObservable)o;
        System.out.println(
                "The current location is: " +
                (double) getJson( observable.getCoFileToMonitor(), "latitude" ) + "," +
                (double) getJson( observable.getCoFileToMonitor(), "longitude")
        );
    }
}
