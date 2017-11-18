package BinghamtonRover.Monitors;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.Observable;
import java.util.Observer;

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
        //System.out.println("InformationObserver [" + cnIDNumber + "] has been updated at " +
        //        convertSecondsToHMmSs((long)arg));
    }

    // Adapted from https://stackoverflow.com/questions/9027317/how-to-convert-milliseconds-to-hhmmss-format
    public static String convertSecondsToHMmSs(long anMilliSeconds)
    {
        return String.format("%1$tH:%1$tM:%1$tS", anMilliSeconds);

    }

    public Object getJson(File aoJSONFile, String asKey)
    {

        /*
         * Here we check if the aoJSONFile file passed exists or not,
         * but it seems like that the try and catch clause will
         * catch the IOexception thrown by reading non existing file
         */
//        if(!aoJSONFile.exists()){
//            System.out.println("The File " + aoJSONFile.getAbsolutePath() + " does not exist");
//            System.exit(1);
//        }

        JSONParser loParser = new JSONParser();

        //instantiate as null to be checked by the method caller
        Object loValue = null;
        try
        {

            //Parse the Json File to a Json Object
            JSONObject aoJSONObj = (JSONObject) loParser.parse(new FileReader(aoJSONFile));

            //If the aoJSONObj don't have the specified Key, loValue = NA
            if(aoJSONObj.containsKey(asKey))
            {
                loValue = aoJSONObj.get(asKey);
            }
            else
            {
                System.out.println("ERROR! cannot find Key: " + asKey + ".");
            }

        }
        catch (IOException e)
        {
            e.printStackTrace();
        }
        catch (ParseException e)
        {
            e.printStackTrace();
        }

        return loValue;
    }

}
