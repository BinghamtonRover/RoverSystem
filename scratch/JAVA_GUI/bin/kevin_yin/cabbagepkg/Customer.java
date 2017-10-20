package cabbagepkg;

import java.util.Observer;
import java.util.Observable;
import java.io.File;

public class Customer implements Observer {

    private Observable stockUpdate;
    private String customer_name;

    public Customer(String name){
        customer_name = name;
    }

    public String getName(){
        return customer_name;
    }

    public void setName(String name){
        customer_name = name;
    }

    //Notifies the customer when cabbages have been restocked
    @Override
    public void update(Observable observable, Object arg){
        stockUpdate = (CabbageStock) observable;
        System.out.println(new File("readme.md").lastModified());
        System.out.println(customer_name+ "! Cabbages have been restocked!");
    }
}