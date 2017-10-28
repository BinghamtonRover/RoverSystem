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
 * The value is in 4 directions divided into two groups: longitude and latitude. But
 * since latitude goes horizontal, I don't really know how we can have N on latitude,
 * and direction E on longitude.
 */

public class DirectionMonitor extends InformationObserver{
}
