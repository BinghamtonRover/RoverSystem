package video;

import javafx.application.Application;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.Parent;
import javafx.stage.Stage;

public class Main extends Application {

    @Override
    public void start(Stage primaryStage){
        try {
            //locate the fxml file
            Parent root = FXMLLoader.load(getClass().getResource("VideoFX.fxml"));
            //set the title of the GUI
            primaryStage.setTitle("Video");
            //creates the GUI with height 800 and widtg 800
            primaryStage.setScene(new Scene(root, 800, 800));
            //makes the GUI visible
            primaryStage.show();

        } catch(Exception e) {
            //prints error
            e.printStackTrace();
        }

}

    public static void main(String[] args){

        launch(args);

    }
}
