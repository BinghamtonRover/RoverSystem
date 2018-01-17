package BinghamtonRover;

import BinghamtonRover.GuiMain.GuiController;
import BinghamtonRover.Monitors.*;
import javafx.application.Application;
import javafx.fxml.FXMLLoader;
import javafx.scene.Parent;
import javafx.scene.Scene;
import javafx.stage.Stage;
import org.opencv.core.Core;

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

        laoObservers.add(new DistanceMonitor());
        laoObservers.add(new BatteryMonitor());
        laoObservers.add(new DirectionMonitor());

        System.out.println(GuiController.class.getResource("."));

        FXMLLoader loLoader = new FXMLLoader(GuiController.class.getResource("./guiScene.fxml"));

        GuiController loController = new GuiController();

        loLoader.setController(loController);

        laoObservers.add(new PressureMonitor(loController));
        laoObservers.add(new TimeMonitor(loController));
        laoObservers.add(new CameraStatusMonitor(loController));
        laoObservers.add(new LocationMonitor(loController));
        laoObservers.add(new TemperatureMonitor(loController));

        FileUpdatingObservable lfuo = new FileUpdatingObservable(gsFile, laoObservers);
        lfuo.startFileMonitoringThread();

        Parent loRoot = (Parent)loLoader.load();
        aoPrimaryStage.setTitle("CameraFeed");
        aoPrimaryStage.setScene(new Scene(loRoot, 700, 400));
        aoPrimaryStage.show();

        //System.exit(0);
    }
}
