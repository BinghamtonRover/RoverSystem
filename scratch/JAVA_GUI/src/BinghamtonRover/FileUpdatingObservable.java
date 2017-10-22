package BinghamtonRover;

import java.io.File;
import java.util.*;

public class FileUpdatingObservable extends Observable
{
    private long cnFileLastUpdatedTime = 0;
    private File coFileToMonitor = null;
    private FileCheckerThread cnCheckerThread = null;

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
        cnFileLastUpdatedTime = 0;
        coFileToMonitor = new File(asFileToWatch);

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
        cnCheckerThread = new FileCheckerThread(this, coFileToMonitor);
        cnCheckerThread.start();
    }

    @Override
    public synchronized void addObserver(Observer aoObserver)
    {
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
}
