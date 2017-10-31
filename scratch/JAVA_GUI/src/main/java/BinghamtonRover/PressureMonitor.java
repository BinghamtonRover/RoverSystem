package BinghamtonRover;

import org.apache.commons.lang3.Validate;
import java.util.Observable;

/**
 * Pressure monitor monitors the air pressure.
 */

public class PressureMonitor extends InformationObserver
{

    public PressureMonitor()
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
                "The Air pressure is: " +
                (double) getJson( loObservable.getCoFileToMonitor(), "pressure" ) + " atm"
        );
    }
}
