package BinghamtonRover.gui;

import javafx.application.Application;
import javafx.fxml.FXMLLoader;
import javafx.geometry.Insets;
import javafx.scene.Scene;
import javafx.scene.layout.Background;
import javafx.scene.layout.BackgroundFill;
import javafx.scene.layout.CornerRadii;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import javafx.stage.Stage;

import java.io.IOException;
import java.net.MalformedURLException;
import java.net.URL;

public class DemoGauges extends Application
{
    private static URL gsFile;
    private static final Color EERIEBLACK = Color.rgb(0x19,0x19,0x19);

    public static void main(String[] args) throws MalformedURLException
    {

        String lsFilePath = "BinghamtonRover/Monitors/python_output.log.json";
        gsFile = (args.length > 0) ? new URL(args[0]) : DemoGauges.class.getClassLoader().getResource(lsFilePath);
        launch(args);
    }

    @Override
    public void start(Stage aoPrimaryStage) throws NullPointerException, IOException
    {

        //Get the path of the FXML file
        URL loFXMLPath = getClass().getClassLoader().getResource("Gui/Blank.fxml");
        System.out.println("FXML Path: " + loFXMLPath.getPath());

        //Initialize a new array of Observer
//        ArrayList<InformationObserver> laoObservers = new ArrayList<>();

        //Start the GUI
        FXMLLoader loLoader = new FXMLLoader(loFXMLPath);
        Pane loRoot = loLoader.load();
        loRoot.setBackground(new Background(new BackgroundFill(EERIEBLACK, CornerRadii.EMPTY, Insets.EMPTY)));
        loRoot.getChildren().addAll(Gauges.HUMIDITY_GAUGE, Gauges.PRESSURE_GAUGE, Gauges.TEMPERATURE_GAUGE, Gauges.CLOCK, Gauges.PRESSURE_GRAPH);
        loRoot.setMinSize(1200.0, 1080);
        Gauges.HUMIDITY_GAUGE.relocate(0, 470);
        Gauges.PRESSURE_GAUGE.relocate(0, 560);
        Gauges.TEMPERATURE_GAUGE.relocate(0, 650);
        Gauges.PRESSURE_GRAPH.relocate(280, 600);
        Gauges.CLOCK.relocate(1080-280, 700);

        aoPrimaryStage.setTitle("Rover Controller");
        aoPrimaryStage.setScene(new Scene(loRoot, 1080,800));

        aoPrimaryStage.show();
        Gauges.StartAnimation();

    }

    @Override
    public void stop() {
        System.exit(0);
    }
}
