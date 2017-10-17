package observerPractice;
import java.util.Observable;
import java.util.Observer;

class Cake extends Observable{
	private int value;
	public Cake(int v){
		value = v;
	}
	
	public void setValue(int v){
		
		if(value != v){
			value = v;
			System.out.println("Value changed to: " + value);
			
			setChanged();
			notifyObservers(v);
		}
	}
}

public class Customer implements Observer{
	
	public static void main(String[] args) {
		Cake watched = new Cake(10);
		Customer watcher = new Customer();
				
		watched.setValue(9);
		watched.addObserver(watcher);
		watched.setValue(8);
		watched.setValue(8);
		watched.setValue(7);		
	}
	
	//called whenever observed object is changed
	public void update(Observable obj, Object arg){
		System.out.println("Update number of cakes available: " + arg);
	}

}
