package observerPractice;
import java.util.Observable;


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
