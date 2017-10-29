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
 * The Temperature Monitor should monitor the temperature around the rover.
 * The New json file shows that the temperature will probably be in Fahrenheit.
 */

public class TemperatureMonitor extends InformationObserver{

    public TemperatureMonitor(){
        super();
    }

    @Override
    public void update(Observable o, Object arg) {
        super.update(o, arg);

        FileUpdatingObservable observable = (FileUpdatingObservable)o;
        System.out.println(
                "The current temperature is: " +
                        (double) getJson( observable.getCoFileToMonitor(), "temperature" ) + " F"
        );
    }
}
