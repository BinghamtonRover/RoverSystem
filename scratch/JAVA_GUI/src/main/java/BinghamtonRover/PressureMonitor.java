package BinghamtonRover;

import org.apache.commons.lang3.Validate;

import java.io.File;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Observable;
import java.util.Observer;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;
import java.io.IOException;
/**
 * Pressure monitor monitors the air pressure.
 */

public class PressureMonitor extends InformationObserver{

    public PressureMonitor(){
        super();
    }

    @Override
    public void update(Observable o, Object arg) {
        super.update(o, arg);

        FileUpdatingObservable observer = (FileUpdatingObservable)o;
        System.out.println(
                "The Air pressure is: " +
                (double) getJson( observer.getCoFileToMonitor(), "pressure" )
        );
    }
}
