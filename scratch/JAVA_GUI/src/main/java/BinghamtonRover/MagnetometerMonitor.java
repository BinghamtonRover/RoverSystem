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
 * The MagnetometerMonitor watches the direction that the rover is facing, it is like a compass.
 * The value is probably in degrees from 0 to 360 with 0 and 360 pointing to North
 */

public class MagnetometerMonitor extends InformationObserver{
}
