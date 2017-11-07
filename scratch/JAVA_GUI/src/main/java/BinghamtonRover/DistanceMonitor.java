package BinghamtonRover;

import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.scene.control.TextField;

import java.util.Observable;

/*
 * The DistanceMonitor should monitor the distance that the rover have traveled.
 * the Json file provided the total distance the rover travelled
 */

public class DistanceMonitor extends InformationObserver
{
    public DistanceMonitor(){
        super();
    }

    @FXML
    private TextField distance_status;

    @Override
    public void update(Observable o, Object arg)
    {
        FileUpdatingObservable loObservable = (FileUpdatingObservable) o;

        double lfDistance = (double) getJson(loObservable.getCoFileToMonitor(), "totalDistanceTraveled");

        System.out.println("Total distance traveled is: " + lfDistance);
        //Update distance monitor on gui
        Platform.runLater(new Runnable()
        {
            @Override public void run()
            {
                distance_status.setText(String.valueOf(lfDistance));
            }
        });
    }
}
