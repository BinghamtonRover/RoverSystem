import java.util.Scanner;
import cabbagepkg.*;
import java.io.File;


public class Demo {

    public static void main(String[] args){

        boolean stock_status;
        Scanner sc = new Scanner(System.in);
        CabbageStock stock = new CabbageStock();
        stock.addObserver(new Customer("Kevin"));
        while(true){
            stock_status = (sc.nextInt() > 0); //check if stock is greater than 0
            stock.setStock(stock_status);  //set stock status
        }
    }
}