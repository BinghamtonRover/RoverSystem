import java.util.Scanner;
import cabbagepkg.*;

public class Demo {

    public static void main(String[] args){

        boolean stock_status;
        Scanner sc = new Scanner(System.in);
        CabbageStock stock = new CabbageStock(false);
        stock.addObserver(new Customer("Kevin"));
        while(true){
            stock_status = (sc.nextInt() > 0) ? true : false; //check if stock is greater than 0
            stock.setStock(stock_status);  //set stock status
        }
    }
}