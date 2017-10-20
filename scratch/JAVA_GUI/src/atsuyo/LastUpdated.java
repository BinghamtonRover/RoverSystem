package observerPractice;
import java.io.File;
import java.text.SimpleDateFormat;
import java.util.Observable;

public class LastUpdated extends Observable{
	private File file;
	private long lastModified;
	
	public LastUpdated(File f) {
		file = f;
		lastModified = file.lastModified();
		
	}
	
	public void check() {
		long temp = file.lastModified();
		if(lastModified != temp) {
			lastModified = temp;
			SimpleDateFormat sdf = new SimpleDateFormat("MM/dd/yyyy HH:mm:ss");
			System.out.println("After Format: " + sdf.format(file.lastModified()));
			
			setChanged();
			notifyObservers();			
		}
	}


}
