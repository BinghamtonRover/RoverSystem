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
 * The DistanceMonitor should monitor the distance that the rover have traveled.
 * The Encode on the wheel of the rover ticks in the sequence(probably) 00,01,11,10,00,01...
 * as the wheel rolls. So there isn't really an actual unit for this, we might need the hardware
 * team to come up with some digital counter to count the numbers of ticks the wheels returns
 * and come up with a ticks/unit distance for us to convert.
 * This monitor will probably take in the tick count from the counter.
 */

public class DistanceMonitor extends InformationObserver{

}
