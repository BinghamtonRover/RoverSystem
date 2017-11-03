import BinghamtonRover.*;
import javafx.application.Application;
import javafx.stage.Stage;

import java.io.IOException;
import java.util.ArrayList;
import java.io.File;

import static javafx.application.Application.launch;

public class DemoRunner extends Application
{
    public void start(String[] args)
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
    public static void main(String[] args)
    {
        launch(args);
    }

    @Override
    public void start(Stage primaryStage) throws Exception {

    }
}
