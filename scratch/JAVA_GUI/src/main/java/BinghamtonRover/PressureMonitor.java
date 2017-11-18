package BinghamtonRover;

import java.util.Observable;
import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.scene.control.Label;
import javafx.scene.control.TextField;

/*
 * Pressure monitor monitors the air pressure.
 */

public class PressureMonitor extends InformationObserver
{
    public PressureMonitor()
    {
        super();
    }

    @FXML private TextField pressure_status;

    @Override
    public void update(Observable o, Object arg)
    {
        FileUpdatingObservable loObservable = (FileUpdatingObservable) o;

        double lfPressure = (double) getJson(loObservable.getCoFileToMonitor(), "pressure");

        System.out.println("The air pressure is: " + lfPressure + "stp");
        //update Pressure Monitor on GUI
        //add FXMLloader from pull request
        Platform.runLater(new Runnable()
        {
            @Override public void run()
            {
                pressure_status.setText(String.valueOf(lfPressure));
            }
        });
    }
}
