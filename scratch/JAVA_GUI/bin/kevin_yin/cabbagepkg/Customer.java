package cabbagepkg;

public class Customer extends Observer {

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
    public void update(){
        System.out.println(customer_name + "! The cabbages have been restocked!");
    }
}