package BinghamtonRover;


import java.util.Observable;


/**
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
        //Call super method to acknowledge user that this Observer has been updated
        super.update(o, arg);

        FileUpdatingObservable loObservable = (FileUpdatingObservable)o;

        //Print out the information that this observer is monitoring
        System.out.println
        (
                "The current location is: " +
                (double) getJson( loObservable.getCoFileToMonitor(), "latitude" ) + "," +
                (double) getJson( loObservable.getCoFileToMonitor(), "longitude")
        );
    }
}
