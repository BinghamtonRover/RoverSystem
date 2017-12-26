package BinghamtonRover.Bluetooth;

import javafx.event.ActionEvent;
import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.TextArea;

import java.util.concurrent.ExecutorService;


public class BluetoothClientGUIController {

    private boolean cbServerStarted = false;
    private SampleSPPClient coClient;
    private ExecutorService coClientRunner;


    @FXML
    private TextArea coMessageBoard;
    @FXML
    private Button coStartQueryBtn;

    @FXML
    public void updateText(String asTextToAppend){
        coMessageBoard.appendText("\n" + asTextToAppend);
    }

    @FXML
    private void StartServer(ActionEvent event)
    {
        //If the Server has not start, run the server
        if (!cbServerStarted) {
            coClient = new SampleSPPClient();


            /**
             * Need to create a runnable which executes the device inquiry process
             * of the SampleSPPClient
             */
//            coClientRunner = Executors.newSingleThreadExecutor();
//            coClientRunner.execute();

            //Set ServerStarted flag and button text.
            this.coStartQueryBtn.setText("Stop Query");
            ServerStarted();
        }
        else if(cbServerStarted)
        {
            this.coClientRunner.shutdown();
            System.out.println("Shutting down the client");

            //Set ServerStarted flag and button text.
            this.coStartQueryBtn.setText("Start Query");
            ServerClosed();
        }


    }

    public void ApplicationCLosed()
    {
        this.coClientRunner.shutdown();
    }

    private void ServerStarted()
    {
        cbServerStarted = true;
    }

    private void ServerClosed()
    {
        cbServerStarted = false;
    }
}
