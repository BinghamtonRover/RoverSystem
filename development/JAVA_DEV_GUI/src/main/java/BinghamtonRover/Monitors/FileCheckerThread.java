package BinghamtonRover.Monitors;

import org.apache.commons.lang3.Validate;

import java.io.File;

public class FileCheckerThread extends Thread
{
    private long cnLastUpdateTime = 0;
    private File coFileToMonitor = null;
    private FileUpdatingObservable coObservable = null;

    public FileCheckerThread(FileUpdatingObservable aoObservable, File aoFileToMonitor)
    {
        // If any args are null, get out of the method
        Validate.notNull(aoObservable,"FileCheckerThread: Observable is null");
        Validate.notNull(aoFileToMonitor, "FileCheckerThread: File is null");

        // If the status file does not exist, kill the Thread
        if (! aoFileToMonitor.exists())
        {
            System.out.println("File " + aoFileToMonitor.getAbsolutePath() + " not found");
            System.exit(1);
        }

        coObservable = aoObservable;
        coFileToMonitor = aoFileToMonitor;
        cnLastUpdateTime = coFileToMonitor.lastModified();
    }

    /**
     * Once you start the thread, the run() method is auto-called. Once this method finishes,
     * the Thread will exit. We happen to have an infinite loop in our run() method, so this
     * Thread will never exit unless an exception is raised.
     */
    @Override
    public void run()
    {
        while (true)
        {
            /*
             * Every time we execute this loop, we need to make sure this file still exists
             * or we'll run across some problems
             */
            if (coFileToMonitor.exists())
            {
                if (cnLastUpdateTime < coFileToMonitor.lastModified())
                {
                    // Alert observers of this update
                    cnLastUpdateTime = coFileToMonitor.lastModified();
                    coObservable.fileWasUpdated(cnLastUpdateTime);
                }

                try
                {
                    /*
                     * We want this thread to check the modification time of the file
                     * ~10 times a second. This will sleep the thread so we achieve
                     * those results.
                     */
                    Thread.sleep(100);
                }
                catch (InterruptedException aException)
                {
                    aException.printStackTrace();
                }
            }
            else
            {
                System.out.println("File " + coFileToMonitor.getAbsolutePath() + " no longer exists");
                break; // This will terminate the thread by ending the run() method
            }
        }
    }
}