package BinghamtonRover.Monitors;

import org.apache.commons.lang3.Validate;

import java.io.File;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Observable;
import java.util.Observer;

public class FileUpdatingObservable extends Observable
{
    private long cnFileLastUpdatedTime = 0;
    private File coFileToMonitor = null;

    public FileUpdatingObservable(String asFileToWatch)
    {
       this(asFileToWatch, new ArrayList<>());
    }

    public FileUpdatingObservable(String asFileToWatch, InformationObserver aoObserver)
    {
        // If aoObserver is null, forward an empty array list. Otherwise, add the single element
        this(asFileToWatch, (aoObserver != null) ? new ArrayList<>(Collections.singletonList(aoObserver)) : new ArrayList<>());
    }

    public FileUpdatingObservable(String asFileToWatch, ArrayList<InformationObserver> aaoObservers)
    {
        // Ensure all parameters are not null
        Validate.notNull(asFileToWatch, "FileUpdatingObservable: String cannot be null");
        Validate.notNull(aaoObservers, "FileUpdatingObservable: ArrayList cannot be null");

        // Cannot throw a NullPointerException since arg will never be null
        coFileToMonitor = new File(asFileToWatch);
        cnFileLastUpdatedTime = 0;

        if (! coFileToMonitor.exists())
        {
            System.out.println("File " + coFileToMonitor.getAbsolutePath() + " does not exist");
            System.exit(1);
        }

        if (! aaoObservers.isEmpty())
        {
            for (Observer aaoObserver : aaoObservers)
            {
                addObserver(aaoObserver);
            }
        }
    }

    public void startFileMonitoringThread()
    {
        FileCheckerThread lnCheckerThread = new FileCheckerThread(this, coFileToMonitor);
        lnCheckerThread.start();
    }

    @Override
    public synchronized void addObserver(Observer aoObserver)
    {
        // Avoid a NPE by ensuring we are not adding a null Observer
        Validate.notNull(aoObserver, "addObserver: Observer is null");
        super.addObserver(aoObserver);
    }

    /**
     * Package-private method. This method should ONLY be called from FileCheckerThread
     * as it will then start the chain of events to let the GUI know that there are
     * new stats available.
     * @param anUpdateTime Time the file was updated
     */
    void fileWasUpdated(long anUpdateTime)
    {
        cnFileLastUpdatedTime = anUpdateTime;
        setChanged();
        notifyObservers(cnFileLastUpdatedTime);
    }

    public File getCoFileToMonitor()
    {
        Validate.notNull(coFileToMonitor, "File to monitor is null");

        if (! coFileToMonitor.exists())
        {
            System.out.println("File " + coFileToMonitor.getAbsolutePath() + " does not exist");
            System.exit(1);
        }

        return coFileToMonitor;
    }
}
