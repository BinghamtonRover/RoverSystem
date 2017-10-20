import java.util.*;

class Server implements Observer {
    public void update(Observable o, Object arg) {
        System.out.println("ring");
    }
    
    public static void main(String[] args) {
        Servable served = new Servable(2000000000);
        Server main2 = new Server();
        
        served.addObserver(main2);
        
        while (true) {
            served.tick();
        }
    }
}
