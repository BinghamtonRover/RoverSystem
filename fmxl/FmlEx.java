import java.io.FileInputStream;
import java.io.IOException;

import javafx.application.Application;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.layout.VBox;
import javafx.stage.Stage;

import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.scene.input.MouseEvent;

import java.net.URL;
import java.util.ResourceBundle;
import javafx.fxml.Initializable;

import javafx.scene.control.Button;
import javafx.scene.control.Label;
import javafx.scene.control.TextArea;
import javafx.scene.text.*;

public class FmlEx implements Initializable {
    @FXML 
    private Text outputTextArea;

    @FXML
    void handleButtonAction(MouseEvent event) {
        // Button was clicked, do something...
        outputTextArea.setText("Button Action\n");
        
        System.out.println("did something!");
    }
    
    @Override
    public void initialize(URL url, ResourceBundle rb) {
        
    } 
}
