import BinghamtonRover.Monitors.*;
import javafx.application.Application;
import javafx.fxml.FXMLLoader;
import javafx.scene.Parent;
import javafx.scene.Scene;
import javafx.stage.Stage;
import org.opencv.core.Core;
import org.opencv.videoio.VideoCapture;

import java.util.ArrayList;

public class DemoRunner extends Application
{
    private static String gsFile;

    // Load in OpenCV3 libraries
    static
    {
        nu.pattern.OpenCV.loadShared();
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }

    public static void main(String[] args)
    {
        gsFile = (args.length > 0) ? args[0] : "./src/main/resources/BinghamtonRover/Monitors/python_output.log.json";
        launch(args);

    }

    @Override
    public void start(Stage aoPrimaryStage) throws Exception
    {
        ArrayList<InformationObserver> laoObservers = new ArrayList<>();

        laoObservers.add(new PressureMonitor());
        laoObservers.add(new LocationMonitor());
        laoObservers.add(new DistanceMonitor());
        laoObservers.add(new BatteryMonitor());
        laoObservers.add(new DirectionMonitor());
        laoObservers.add(new TimeMonitor());
        laoObservers.add(new TemperatureMonitor());
        laoObservers.add(new CameraStatusMonitor());

        FileUpdatingObservable lfuo = new FileUpdatingObservable(gsFile, laoObservers);
        lfuo.startFileMonitoringThread();

        System.out.println(getClass().getResource("."));

        /*Parent loRoot = FXMLLoader.load(VideoCapture.class.getResource("CameraFeed.fxml"));
        aoPrimaryStage.setTitle("CameraFeed");
        aoPrimaryStage.setScene(new Scene(loRoot, 500, 500));
        aoPrimaryStage.show();*/

        System.exit(0);
    }
}
