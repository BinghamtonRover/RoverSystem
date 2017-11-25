package BinghamtonRover.Bluetooth;

import javafx.application.Application;
import javafx.fxml.FXMLLoader;
import javafx.scene.Group;
import javafx.scene.Parent;
import javafx.scene.Scene;
import javafx.stage.Stage;
import org.apache.commons.lang3.Validate;

import java.io.IOException;
import java.net.URL;

public class BluetoothGUI extends Application {

    private FXMLLoader coLoader;

    public static void main (String args[])
    {
        launch(args);
    }


    @Override
    public void start(Stage primaryStage)
    {
        URL loFXMLurl = getClass().getResource("/Bluetooth/BluetoothGUI.fxml");
        System.out.println(loFXMLurl.getPath());
        try
        {
            coLoader = new FXMLLoader(loFXMLurl);
            Parent root = (Parent) coLoader.load(loFXMLurl);
            primaryStage.setTitle("Bluetooth Connection");
            primaryStage.setScene(new Scene(root));
            primaryStage.show();
        }
        catch(IOException e)
        {
            e.printStackTrace();
        }

        SampleSPPServer sampleSPPServer = new SampleSPPServer();
        sampleSPPServer.run();
    }

//    public static void main (String args[]){
//        BluetoothGUI GUI = new BluetoothGUI();
//
//        GUI.run();
//    }
}
