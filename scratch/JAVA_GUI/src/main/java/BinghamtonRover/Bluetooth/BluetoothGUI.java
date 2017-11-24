package BinghamtonRover.Bluetooth;

import javafx.application.Application;
import javafx.fxml.FXMLLoader;
import javafx.scene.Group;
import javafx.scene.Parent;
import javafx.scene.Scene;
import javafx.stage.Stage;

import java.io.IOException;

public class BluetoothGUI extends Application implements Runnable{

    //Might need to put this main method into its own thread

    @Override
    public void run() {
        launch();
    }

    @Override
    public void start(Stage primaryStage) {
        try
        {
            FXMLLoader loader = new FXMLLoader();
            Parent root = loader.load(getClass().getResource("/Bluetooth/BluetoothGUI.fxml"));
                    primaryStage.setTitle("Bluetooth Connection");
            primaryStage.setScene(new Scene(root));
            primaryStage.show();
        }
        catch(IOException e)
        {
            e.printStackTrace();
        }
    }

    public static void main (String args[]){
        BluetoothGUI GUI = new BluetoothGUI();

        GUI.run();
    }
}
