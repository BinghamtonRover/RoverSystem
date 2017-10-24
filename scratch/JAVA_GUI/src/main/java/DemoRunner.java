import BinghamtonRover.FileUpdatingObservable;
import BinghamtonRover.InformationObserver;

import java.util.ArrayList;

public class DemoRunner
{
    public static void main(String[] args)
    {
        String lsFile = (args.length > 0) ? args[0] : "./demo.txt";

        ArrayList<InformationObserver> laoObservers = new ArrayList<>();

        for (int i = 0; i < 10; i++)
        {
            laoObservers.add(new InformationObserver());
        }

        FileUpdatingObservable lfuo = new FileUpdatingObservable(lsFile, laoObservers);

        lfuo.startFileMonitoringThread();
    }
}
