import BinghamtonRover.*;

import java.io.IOException;
import java.util.ArrayList;
import java.io.File;

public class DemoRunner
{
    public static void main(String[] args)
    {
        String lsFile = (args.length > 0) ? args[0] : "./src/main/files/python_output.log.json";

        ArrayList<InformationObserver> laoObservers = new ArrayList<>();

        laoObservers.add(new PressureMonitor());
        laoObservers.add(new LocationMonitor());
        laoObservers.add(new DistanceMonitor());
        laoObservers.add(new BatteryMonitor());
        laoObservers.add(new DirectionMonitor());
        laoObservers.add(new TimeMonitor());
        laoObservers.add(new TemperatureMonitor());
        laoObservers.add(new CameraStatusMonitor());

        FileUpdatingObservable lfuo = new FileUpdatingObservable(lsFile, laoObservers);
        lfuo.startFileMonitoringThread();
    }
}
