package cabbagepkg;

import java.util.List;
import java.util.ArrayList;

public class CabbageStock {

    private List<Customer> observers = new ArrayList<Customer>();
    private boolean inStock;

    //check stocks status
    public boolean checkStock(){
        return inStock;
    }

    //set stock status
    public void setStock(boolean status){
        inStock = status;
    }

    //adds a new customer who want to be notified to observers 
    public void newCustomer(Customer cust){
        observers.add(cust);
    }

    public void removeCustomer(String name){
        observers.remove(name);
    }

    //notifies all the customers who are waiting for cabbages to be restocked
    public void sendNotifications(){
        for(Observer cust: observers){
            cust.update();
        }
    }
}


