package BinghamtonRover.Video;

import javafx.application.Application;
import javafx.fxml.FXMLLoader;
import javafx.scene.Parent;
import javafx.scene.Scene;
import javafx.stage.Stage;
import org.opencv.core.Core;

public class Main extends Application
{

    // Load in OpenCV3 libraries
    static { System.loadLibrary(Core.NATIVE_LIBRARY_NAME); }

    @Override
    public void start(Stage aoPrimaryStage) throws Exception
    {
        Parent loRoot = FXMLLoader.load(getClass().getResource("CameraFeed.fxml"));
        aoPrimaryStage.setTitle("CameraFeed");
        aoPrimaryStage.setScene(new Scene(loRoot, 500, 500));
        aoPrimaryStage.show();
    }


    public static void main(String[] args)
    {
        launch(args);
    }
}
