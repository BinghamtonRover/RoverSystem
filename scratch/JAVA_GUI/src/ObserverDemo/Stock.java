package ObserverDemo;

import java.util.ArrayList;
import java.util.Observable;
import java.util.Observer;
import java.io.File;
import java.text.SimpleDateFormat;

public class Stock extends Observable {

    /******** private fields ********/
    private ArrayList<Observer> observers;
    private final String stockName;
    private double stockPrice;

    private final File theStockFile;

    /******** Constructor ********/
    public Stock(String stockName, double stockPrice) {
        this.observers = new ArrayList<>();
        this.stockName = stockName;
        this.stockPrice = stockPrice;

        theStockFile = new File("url");
    }

    @Override
    public synchronized void addObserver(Observer o) {
        observers.add(o);
    }

    @Override
    public synchronized void deleteObserver(Observer o) {
        int index = observers.indexOf(o);
        observers.remove(index);
    }

    @Override
    public void notifyObservers(Object lastModDate) {
        for (Observer obs: observers){
            obs.update(this, lastModDate);
        }
    }

    public void setPrice(double stockPrice) {
        this.stockPrice = stockPrice;
        notifyObservers();
    }






    public String getName() {
        return stockName;
    }

    public double getPrice() {
        return stockPrice;
    }

    @Override
    public synchronized int countObservers() {
        return observers.size();
    }
}
