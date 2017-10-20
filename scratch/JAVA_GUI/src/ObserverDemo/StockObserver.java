package ObserverDemo;

import java.util.ArrayList;
import java.util.Observable;
import java.util.Observer;

public class StockObserver implements Observer{

    /******** private fields ********/
    private String name;
    private ArrayList<Observable> stocks;


    /******** Constructor ********/
    public StockObserver(String name) {
        this.stocks = new ArrayList<>();
        this.name = name;
    }

    public void watchStock(Stock stock){
        stocks.add(stock);
        stock.addObserver(this);
        System.out.println(name + " is watching " + stock.getName() + "\n");
    }

    public void removeStock(Stock stock){
        stocks.remove(stock);
        stock.deleteObserver(this);
        System.out.println(name + " gave up on " + stock.getName() + "\n");
    }

    @Override
    public void update(Observable o, Object lastModDate) {
        Stock stock = (Stock)o;
        System.out.println(name + " see that " + stock.getName()+ " has updated!");
        System.out.println("New price: " + stock.getPrice() + "\n");
        System.out.println("Last Modified: " + (long)lastModDate);
    }

    public String getName() {
        return name;
    }

    @Override
    public String toString() {
        String str = name +  " is Watching ";

        for (Observable obs: stocks){
            str += "\n" + ((Stock)obs).getName();
        }
        return str + "\n";
    }

}
