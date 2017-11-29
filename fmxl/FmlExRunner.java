import java.io.FileInputStream;
import java.io.IOException;

import javafx.application.Application;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.layout.VBox;
import javafx.stage.Stage;

public class FmlExRunner extends Application {
    public static void main(String[] args) {
        System.out.println("yo");
    
        Application.launch(args);
    }
    
    @Override
    public void start(Stage stage) {
        try {
            System.out.println(System.getProperty("user.dir"));
        
            FXMLLoader loader = new FXMLLoader();
            
            

            FileInputStream fxmlStream = new FileInputStream("scene.fxml");

            System.out.println("yay");

            VBox root = (VBox) loader.load(fxmlStream);

            Scene scene = new Scene(root);
            stage.setScene(scene);
            stage.setTitle("A simple FXML Example");
            stage.show();
        } catch (Exception e) {
            System.out.println(e);
        }
    }
}
