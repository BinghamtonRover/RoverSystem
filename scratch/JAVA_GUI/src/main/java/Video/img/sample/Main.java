package Video.img.sample;

import javafx.application.Application;
import javafx.fxml.FXMLLoader;
import javafx.scene.Parent;
import javafx.scene.Scene;
import javafx.stage.Stage;
import org.opencv.core.Core;

public class Main extends Application {

    static {System.loadLibrary(Core.NATIVE_LIBRARY_NAME);}

    @Override
    public void start(Stage primaryStage) throws Exception{
        Parent root = FXMLLoader.load(getClass().getResource("sample.fxml"));
        primaryStage.setTitle("CameraFeed");
        primaryStage.setScene(new Scene(root, 1440, 900));
        primaryStage.show();
    }


    public static void main(String[] args)
    {
        launch(args);
    }
}
