import javafx.application.Application;
import javafx.fxml.FXMLLoader;
import javafx.scene.Parent;
import javafx.scene.Scene;
import javafx.stage.Stage;

public class DemoRunner extends Application
{
    @Override
    public void start(Stage primaryStage)throws Exception{
        /*
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
        */
        //having trouble getting GUI to show

        Parent root = FXMLLoader.load(getClass().getResource("UpdateGui.fxml"));
        primaryStage.setTitle("GUI");
        primaryStage.setScene(new Scene(root, 800, 800));
        primaryStage.show();

    }
    public static void main(String[] args)
    {
        launch(args);
    }

}
