import java.util.Scanner;
import cabbagepkg.*;

public class Demo {

    public static void main(String[] args){

        boolean stock_status;
        Scanner sc = new Scanner(System.in);
        CabbageStock stock = new CabbageStock();
        stock.setStock(false);
        stock.newCustomer(new Customer("Kevin")); //adds a customer who wants to be notified, aka Observant.
        stock.newCustomer(new Customer("Maria"));
        stock.newCustomer(new Customer("Angel"));
        stock.removeCustomer("Zach"); //removes zach from the notification list
        while(true){
            stock_status = (sc.nextInt() > 0) ? true : false; //check if stock is greater than 0
            stock.setStock(stock_status);  //set stock status
            if(stock.checkStock() == true) stock.sendNotifications(); //update all customers
        }
    }
}