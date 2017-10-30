package BinghamtonRover;

import org.apache.commons.lang3.Validate;
import java.util.Observable;

/**
 * Pressure monitor monitors the air pressure.
 */

public class PressureMonitor extends InformationObserver
{

    public PressureMonitor(){
        super();
    }

    @Override
    public void update(Observable o, Object arg)
    {
        super.update(o, arg);

        FileUpdatingObservable observable = (FileUpdatingObservable)o;
        System.out.println(
                "The Air pressure is: " +
                (double) getJson( observable.getCoFileToMonitor(), "pressure" ) + " atm"
        );
    }
}
