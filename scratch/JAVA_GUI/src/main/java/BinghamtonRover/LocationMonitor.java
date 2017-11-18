package BinghamtonRover;

import javafx.application.Platform;
import javafx.fxml.FXML;

import java.awt.*;
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

    @FXML
    private TextField location_status;

    @Override
    public void update(Observable o, Object arg)
    {
        FileUpdatingObservable loObservable = (FileUpdatingObservable) o;

        double lfLatitude = (double) getJson(loObservable.getCoFileToMonitor(), "latitude");
        double lfLongitude = (double) getJson(loObservable.getCoFileToMonitor(), "longitude");

        System.out.println("The current location is: " + lfLatitude + ", " + lfLongitude);
        //Update Location Monitor on GUI
        Platform.runLater(new Runnable()
        {
            @Override public void run()
            {
                location_status.setText(String.valueOf(lfLatitude) + String.valueOf(lfLongitude));
            }
        });
    }
}
