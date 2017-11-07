package BinghamtonRover;

import javafx.application.Platform;
import javafx.fxml.FXML;

import java.awt.*;
import java.util.Observable;


/*
 * The BatteryMonitor watches the percentage battery remain on the rover.
 * the battery status will be a number between 0-100.
 */

public class BatteryMonitor extends InformationObserver
{

    public BatteryMonitor()
    {
        super();
    }

    @FXML
    private TextField battery_status;

    @Override
    public void update(Observable o, Object arg)
    {
        FileUpdatingObservable loObservable = (FileUpdatingObservable) o;

        String lbBattery = (String) getJson(loObservable.getCoFileToMonitor(), "batteryLevel");

        System.out.println("The current Battery percentage is: lbBattery" + lbBattery + "%");

        //Update battery status on gui
        Platform.runLater((Runnable) () -> battery_status.setText(lbBattery));
    }
}
