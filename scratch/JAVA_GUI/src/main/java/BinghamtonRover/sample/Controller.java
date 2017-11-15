package sample;

//import javafx.event.EventHandler;


import java.beans.EventHandler;
import java.net.URL;
import java.util.ResourceBundle;
import javafx.fxml.FXML;
import javafx.scene.control.Label;
import javafx.scene.input.MouseEvent;
import javafx.scene.paint.Color;
import javafx.scene.text.Font;

import javax.swing.*;

public class Controller {
    @FXML
    private Label goClick_label;

    @FXML
    public void change_text() { //MouseEvent event
        goClick_label.setText(casArr[cnIndex]);
        cnIndex++;
        cnIndex= cnIndex%3;
    }

    private String [] casArr = {"1st","2nd","3rd"};
    private int cnIndex = 0;

    @FXML
    void initialize() {
        assert goClick_label != null : "fx:id=\"goclick_label\" was not injected: check your FXML file 'Untitled'.";

        goClick_label.onMouseClickedProperty().set(new javafx.event.EventHandler<MouseEvent>() {
            @Override
            public void handle(MouseEvent event) {
                change_text();
            }
        });

    }


}
