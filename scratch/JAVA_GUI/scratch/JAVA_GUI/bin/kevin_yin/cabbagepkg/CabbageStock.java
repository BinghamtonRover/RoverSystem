package cabbagepkg;

import java.util.Observable;

public class CabbageStock extends Observable{

    //set stock status
    public void setStock(boolean status){
        setChanged(); //can call hasChanged() method to see whether this observable has changed
        if(status) notifyObservers(); //notifies customers if the stock of cabbages have been replenished
    }

}


