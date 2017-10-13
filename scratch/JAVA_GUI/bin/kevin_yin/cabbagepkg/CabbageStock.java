package cabbagepkg;

import java.util.Observable;

public class CabbageStock extends Observable{

    private boolean inStock;

    public CabbageStock(boolean status){
        inStock = status;
    }

    //set stock status
    public void setStock(boolean status){
        inStock = status;
        setChanged(); //can call hasChanged() method to see whether this observable has changed
        if(status) notifyObservers(); //notifies customers if the stock of cabbages have been replenished
    }

}


