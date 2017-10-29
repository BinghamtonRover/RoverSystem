package BinghamtonRover;

import java.util.Observable;

import org.json.simple.JSONObject;
import org.json.simple.JSONArray;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.Iterator;

/**
 * THe BatteryMonitor watches the percentage battery remain on the rover.
 * the battery status will be a number between 0-100.
 */

public class BatteryMonitor extends InformationObserver {

    public BatteryMonitor(){
        super();
    }

    @Override
    public void update(Observable o, Object arg) {
        super.update(o, arg);

        FileUpdatingObservable observer = (FileUpdatingObservable)o;
        System.out.println(
                "The current Battery percentage is: " +
                (double) getJson( observer.getCoFileToMonitor(), "batteryLevel" )
        );
    }
}
