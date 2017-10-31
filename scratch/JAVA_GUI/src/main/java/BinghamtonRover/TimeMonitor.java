package BinghamtonRover;

import java.text.ParseException;
import java.util.Date;
import java.util.Observable;
import java.text.SimpleDateFormat;
import java.util.Calendar;

/**
 * The time written in the JSON file can serve as an ping measure of the delay after
 * comparing it with the current time on the machine.
 */

public class TimeMonitor extends InformationObserver
{

    private SimpleDateFormat coDateFormat;

    /**
     * Only use static method getInstance() to gets a calendar using the default time zone and locale
     * Therefore no instance assignment or instantiation.
     */
    private Calendar lcCalendar;

    public TimeMonitor()
    {
        super();
        //The pattern of SDF must match to that of the currentTime value inside the JSON file
        coDateFormat = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS");
    }

    @Override
    public void update(Observable o, Object arg)
    {
        //Call super method to acknowledge user that this Observer has been updated
        super.update(o, arg);

        FileUpdatingObservable loObservable = (FileUpdatingObservable)o;
        String lsJsonDate = (String) getJson( loObservable.getCoFileToMonitor(), "currentTime" );

        Date loFileDate = null;
        Date loCurrentDate = lcCalendar.getInstance().getTime();


        //Parse the String obtained from json to a Date
        try
        {
            loFileDate = coDateFormat.parse(lsJsonDate);
        }
        catch (ParseException e)
        {
            e.printStackTrace();
        }

        //We don't want the FileDate to be null
        assert loFileDate != null;

        /**
         * I used the current time to compare with the time written on the json
         * maybe we can just use the file last update time.
         */
        long lnTimeDiff = loCurrentDate.getTime() - loFileDate.getTime();

        //Print out the information that this observer is monitoring
        System.out.println
        (
                "The time of the file update is: " +
                convertSecondsToHMmSs(loFileDate.getTime()) + "\n" + "The current time is: " +
                convertSecondsToHMmSs(loCurrentDate.getTime()) + "\n" + "The time delay is: " +
                lnTimeDiff + " ms"
        );
    }
}
