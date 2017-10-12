package cabbagepkg;

import java.util.List;
import java.util.ArrayList;

public class CabbageStock {

    private List<Customer> observers = new ArrayList<Customer>();
    private boolean inStock;

    public CabbageStock(boolean status){
        inStock = status;
    }
    //check stocks status
    public boolean checkStock(){
        return inStock;
    }

    //set stock status
    public void setStock(boolean status){
        inStock = status;
    }

    //adds a new customer who want to be notified to observers 
    public void addObserver(Customer cust){
        observers.add(cust);
    }

    public void removeObserver(String name){
        observers.remove(name);
    }

    //notifies all the customers who are waiting for cabbages to be restocked
    public void notifyObservers(){
        for(Observer cust: observers){
            cust.update();
        }
    }
}


