package sample;

import javafx.application.Application;
import javafx.event.EventHandler;
import javafx.event.ActionEvent;
import javafx.fxml.FXMLLoader;
import javafx.scene.Parent;
import javafx.scene.Scene;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.StackPane;
import javafx.stage.Stage;
import javafx.scene.control.Label;
import javafx.scene.control.Button;
public class Main extends Application {
Label goLabel;
//create a label
int giCntr = 0;
//create a counter to cycle through the text
    String[] test = {"test1", "different text"};
    //hold sample text in an array
    @Override
    public void start(Stage primaryStage) throws Exception {
        Parent coRoot = FXMLLoader.load(getClass().getResource("sample.fxml"));
        primaryStage.setTitle("Hello World");
        primaryStage.setScene(new Scene(coRoot, 300, 275));
        primaryStage.show();
        //starting bullshit Scenebuilder does (changed root to coRoot)
        goLabel = new Label("Hope this works");
        //set label txt to Hope this works
        goLabel.setOnMouseClicked(new EventHandler<MouseEvent>() {
            //if it detects a mouseclick on the label handle it
            @Override
            public void handle(MouseEvent event) {
                giCntr++;
                giCntr %= test.length;
                //shift one to the right in the array
                goLabel.setText(test[giCntr]);
                //set label text to the next thing
            }
        });
        StackPane layout = new StackPane();
        layout.getChildren().add(goLabel);
        //put the label on the stackpane
        Scene coScene = new Scene(layout, 300, 250);
        //make scene
        primaryStage.setScene(coScene);
        //set scene
        primaryStage.show();


    }

    public static void main(String[] args) {
        launch(args);
    }
}
