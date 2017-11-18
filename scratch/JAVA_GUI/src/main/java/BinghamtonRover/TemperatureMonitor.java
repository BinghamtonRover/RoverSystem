package BinghamtonRover;

import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.scene.control.TextField;

import javax.xml.soap.Text;
import java.util.Observable;

/*
 * The Temperature Monitor should monitor the temperature around the rover.
 * The New json file shows that the temperature will probably be in Fahrenheit.
 */

public class TemperatureMonitor extends InformationObserver
{
    public TemperatureMonitor()
    {
        super();
    }

    @FXML
    private TextField temperature_status;

    @Override
    public void update(Observable o, Object arg)
    {
        FileUpdatingObservable loObservable = (FileUpdatingObservable) o;

        double lfTemperature = (double) getJson(loObservable.getCoFileToMonitor(), "temperature");

        System.out.println("The current temperature is: " + lfTemperature + "°F");
        //Update temperature monitor on GUI
        Platform.runLater(new Runnable()
        {
            @Override public void run()
            {
                temperature_status.setText(String.valueOf(lfTemperature));
            }
        });
    }
}
