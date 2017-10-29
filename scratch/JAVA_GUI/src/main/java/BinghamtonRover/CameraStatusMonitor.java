package BinghamtonRover;

import javax.sound.sampled.Line;
import java.util.Observable;

public class CameraStatusMonitor extends InformationObserver{

    public CameraStatusMonitor(){
        super();
    }

    @Override
    public void update(Observable o, Object arg) {
        super.update(o, arg);

        FileUpdatingObservable observable = (FileUpdatingObservable)o;
        System.out.println(
                "The Camera is: " +
                (String) getJson( observable.getCoFileToMonitor(), "cameraStatus" )
        );
    }
}
