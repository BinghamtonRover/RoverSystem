package BinghamtonRover.Monitors;

import BinghamtonRover.GuiMain.GuiController;

import java.util.Observable;

public class CameraStatusMonitor extends InformationObserver
{

    private GuiController coController;

    public CameraStatusMonitor()
    {
        super();
    }

    public CameraStatusMonitor(GuiController loController)
    {
        super();

        coController=loController;
    }

    @Override
    public void update(Observable o, Object arg)
    {
        FileUpdatingObservable loObservable = (FileUpdatingObservable) o;

        String lsCamera = (String) getJson(loObservable.getCoFileToMonitor(), "cameraStatus");

        if (coController != null) coController.updateStatus("The Camera is: " + lsCamera );
    }
}
