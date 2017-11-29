package BinghamtonRover.Monitors;

import BinghamtonRover.GuiMain.GuiController;

import java.text.ParseException;
import java.util.Date;
import java.util.Observable;
import java.text.SimpleDateFormat;
import java.util.Calendar;

/*
 * The time written in the JSON file can serve as an ping measure of the delay after
 * comparing it with the current time on the machine.
 */

public class TimeMonitor extends InformationObserver
{

    private SimpleDateFormat coDateFormat;
    private GuiController coController;

    /**
     * Only use static method getInstance() to gets a calendar using the default time zone and locale
     * Therefore no instance assignment or instantiation.
     */

    public TimeMonitor()
    {
        super();

        //The pattern of SDF must match to that of the currentTime value inside the JSON file
        coDateFormat = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss");
    }

    public TimeMonitor(GuiController loController)
    {
        super();

        coDateFormat = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss");
        coController=loController;
    }

    @Override
    public void update(Observable o, Object arg)
    {
        FileUpdatingObservable loObservable = (FileUpdatingObservable) o;

        String lsJsonDate = (String) getJson(loObservable.getCoFileToMonitor(), "currentTime");

        Date loFileDate = null;
        Date loCurrentDate = Calendar.getInstance().getTime();


        //Parse the String obtained from json to a Date
        try
        {
            loFileDate = coDateFormat.parse(lsJsonDate);
        }
        catch (ParseException e)
        {
            e.printStackTrace();
            System.exit(1);
        }

        /*
         * I used the current time to compare with the time written on the json
         * maybe we can just use the file last update time.
         */

        String lsFileDate = convertSecondsToHMmSs(loFileDate.getTime());
        String lsCurrentDate = convertSecondsToHMmSs(loCurrentDate.getTime());
        long lnTimeDiff = loCurrentDate.getTime() - loFileDate.getTime();

        System.out.println("Time of file update is: " + lsFileDate);
        System.out.println("Current time is: " + lsCurrentDate);
        System.out.println("Time delay is: " + lnTimeDiff);

        if (coController != null) coController.updateTime("Current time is: " + lsCurrentDate );
    }
}
