package BinghamtonRover;


import javafx.application.Platform;
import javafx.fxml.FXML;

import java.awt.*;
import java.util.Observable;

public class CameraStatusMonitor extends InformationObserver
{

    public CameraStatusMonitor()
    {
        super();
    }

    @FXML
    private TextField camera_status;

    @Override
    public void update(Observable o, Object arg)
    {
        FileUpdatingObservable loObservable = (FileUpdatingObservable) o;

        String lsCamera = (String) getJson(loObservable.getCoFileToMonitor(), "cameraStatus");

        System.out.println("The Camera is: " + lsCamera);

        //Update camera monitor on gui
        Platform.runLater(new Runnable()
        {
            @Override public void run()
            {
                camera_status.setText(lsCamera);
            }
        });
    }
}
