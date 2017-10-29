package BinghamtonRover;

import org.omg.CORBA.Current;

import java.io.File;
import java.text.ParseException;
import java.util.Date;
import java.util.Observable;
import java.text.SimpleDateFormat;
import java.util.Calendar;

/**
 * The time written in the JSON file can serve as an ping measure of the delay after
 * comparing it with the current time on the machine.
 */

public class TimeMonitor extends InformationObserver{

    private SimpleDateFormat dateFormat;
    private Calendar cal;

    public TimeMonitor(){
        super();
        dateFormat = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss");
    }

    @Override
    public void update(Observable o, Object arg) {
        super.update(o, arg);

        FileUpdatingObservable observable = (FileUpdatingObservable)o;
        String jsonDate = (String) getJson( observable.getCoFileToMonitor(), "currentTime" );

        Date FileDate = null;
        Date CurrentDate = cal.getInstance().getTime();


        //Parse the String obtained from json to a Date
        try {
            FileDate = dateFormat.parse(jsonDate);
        }
        catch (ParseException e){
            e.printStackTrace();
        }

        //We don't want the FileDate to be null
        assert FileDate != null;

        /**
         * I used the current time to compare with the time written on the json
         * maybe we can just use the file last update time.
         */

        long timeDiff = CurrentDate.getTime() - FileDate.getTime();

        System.out.println(
                "The time of the file update is: " +
                convertSecondsToHMmSs(FileDate.getTime()) + "\n" + "The current time is: " +
                convertSecondsToHMmSs(CurrentDate.getTime()) + "\n" + "The time delay is: " +
                timeDiff + " ms"
        );



    }
}
