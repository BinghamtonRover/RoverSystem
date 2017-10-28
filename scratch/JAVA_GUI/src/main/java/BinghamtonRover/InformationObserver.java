package BinghamtonRover;

import org.apache.commons.lang3.Validate;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import java.io.File;
import java.io.FileReader;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Observable;
import java.util.Observer;
import java.util.stream.IntStream;

/**
 * This class will be the super() for all gauges/gears on the GUI. From there,
 * we can update them with all the useful information while retaining a sense
 * of order in who/what is listening in on the Observable and maintain proper
 * boundary checking.
 */
public class InformationObserver implements Observer
{
    private static int gnCounter = 0;
    private int cnIDNumber = 0;

    // Establish a unique ID to each instance of this class
    public InformationObserver()
    {
        cnIDNumber = gnCounter++;
    }

    /**
     * When the Observable has a change, this method will be called. The arguments are,
     * for the most part, not useful as we'll be reading the file ourselves.
     * @param o Observable triggering us
     * @param arg Time we were updated at
     */
    @Override
    public void update(Observable o, Object arg)
    {
        System.out.println("InformationObserver [" + cnIDNumber + "] has been updated at " +
                convertSecondsToHMmSs((long)arg));
    }

    // Adapted from https://stackoverflow.com/questions/9027317/how-to-convert-milliseconds-to-hhmmss-format
    public static String convertSecondsToHMmSs(long anMilliSeconds)
    {
        return String.format("%1$tH:%1$tM:%1$tS", anMilliSeconds);

    }

    protected Object getJson(File JSON, String key){

        /**
         * Here we check if the JSON file passed exists or not,
         * but it seems like that the try and catch clause will
         * catch the IOexception thrown by reading non existing file
         */
//        if(!JSON.exists()){
//            System.out.println("The File " + JSON.getAbsolutePath() + " does not exist");
//            System.exit(1);
//        }

        JSONParser parser = new JSONParser();
        Object value = "NA";
        try {

            JSONObject json = (JSONObject) parser.parse(new FileReader(JSON));

            //If the json don't have the specified key, value = NA
            if(json.containsKey(key)){
                value = json.get(key);
            }
            else {
                System.out.println("Error, cannot find a key");
            }

        } catch (IOException e) {
            e.printStackTrace();
        } catch (ParseException e) {
            e.printStackTrace();
        }

        return value;
    }

}
