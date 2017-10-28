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
 * The location Monitor will read the longitude and latitude from the Json file.
 */

public class LocationMonitor extends InformationObserver{

    public LocationMonitor(){
        super();
    }

    @Override
    public void update(Observable o, Object arg) {
        super.update(o, arg);

        FileUpdatingObservable obserbvable = (FileUpdatingObservable)o;
        System.out.println(
                "The current location is: " +
                (double) getJson( obserbvable.getCoFileToMonitor(), "latitude" ) + "," +
                (double) getJson( obserbvable.getCoFileToMonitor(), "longitude")
        );
    }
}
