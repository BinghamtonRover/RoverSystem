package BinghamtonRover;

import javafx.application.Platform;
import javafx.fxml.FXML;

import java.awt.*;
import java.util.Observable;

/*
 * Assume this class monitors the target's relative direction to the rover.
 */

public class DirectionMonitor extends InformationObserver
{
    public DirectionMonitor()
    {
        super();
    }

    @FXML
    private TextField direction_status;

    @Override
    public void update(Observable o, Object arg)
    {
        FileUpdatingObservable loObservable = (FileUpdatingObservable) o;

        String lsLatitude = (String) getJson(loObservable.getCoFileToMonitor(), "latitudeDirection");
        String lsLongitude = (String) getJson(loObservable.getCoFileToMonitor(), "longitudeDirection");

        System.out.println("The target is at " + lsLatitude + lsLongitude + " Direction");
        //update direction monitor on gui
        Platform.runLater(new Runnable()
        {
            @Override public void run() {
                direction_status.setText(lsLatitude + lsLongitude);
            }
        });
    }
}
