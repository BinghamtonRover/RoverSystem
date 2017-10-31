package BinghamtonRover;


import java.util.Observable;

public class CameraStatusMonitor extends InformationObserver
{

    public CameraStatusMonitor()
    {
        super();
    }

    @Override
    public void update(Observable o, Object arg)
    {
        FileUpdatingObservable loObservable = (FileUpdatingObservable) o;

        String lsCamera = (String) getJson(loObservable.getCoFileToMonitor(), "cameraStatus");

        System.out.println("The Camera is: " + lsCamera);
    }
}
