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
 * The CompassMonitor watches the direction that the rover is facing, it is like a compass.
 * Assume this class monitors the target's relative direction to the rover.
 */

public class DirectionMonitor extends InformationObserver{

    public DirectionMonitor(){
        super();
    }

    @Override
    public void update(Observable o, Object arg) {
        super.update(o, arg);

        FileUpdatingObservable observer = (FileUpdatingObservable)o;
        System.out.println(
                "The target is at " +
                (String) getJson( observer.getCoFileToMonitor(), "latitudeDirection" ) +
                (String) getJson( observer.getCoFileToMonitor(), "longitudeDirection") +
                " Direction"
        );
    }
}
