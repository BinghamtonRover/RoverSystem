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
        //Call super method to acknowledge user that this Observer has been updated
        super.update(o, arg);

        //Print out the information that this observer is monitoring
        FileUpdatingObservable loObservable = (FileUpdatingObservable)o;
        System.out.println
        (
                "The Camera is: " +
                getJson( loObservable.getCoFileToMonitor(), "cameraStatus" )
        );
    }
}
