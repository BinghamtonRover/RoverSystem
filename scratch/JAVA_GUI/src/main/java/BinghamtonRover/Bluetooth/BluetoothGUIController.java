package BinghamtonRover.Bluetooth;

import javafx.fxml.FXML;
import javafx.scene.control.TextArea;


public class BluetoothGUIController {

    @FXML
    private TextArea coMessageBoard;


    public void updateText(String asTextToAppend){
        coMessageBoard.appendText("\n" + asTextToAppend);
    }

}
