package com.github.zeldazach.binghamtonrover;

import com.github.zeldazach.binghamtonrover.controller.ControllerHandler;
import com.github.zeldazach.binghamtonrover.controller.ControllerState;
import com.github.zeldazach.binghamtonrover.gui.DisplayApplication;
import com.github.zeldazach.binghamtonrover.networking.Manager;
import com.github.zeldazach.binghamtonrover.networking.PacketControl;
import com.github.zeldazach.binghamtonrover.networking.ControllerUpdater;
import javafx.application.Application;

import java.net.SocketException;
import java.net.UnknownHostException;

public class BaseStation {

    public static String baseStationBindAddress;
    public static int baseStationBindPort;

    public static String roverAddress;
    public static int roverPort;

    /**
     * The entry point for the base station control program.
     * @param args Command-line arguments.
     */
    public static void main(String[] args) {
        baseStationBindAddress = args[1];
        baseStationBindPort = Integer.parseInt(args[2]);
        roverAddress = args[3];
        roverPort = Integer.parseInt(args[4]);

        ControllerHandler.init();

        Manager networkManager = null;
        try {
            networkManager = new Manager(baseStationBindAddress, baseStationBindPort, 5);
        } catch (SocketException e) {
            System.err.println("Failed to open network UDP socket: " + e.getMessage());
            System.exit(1);
        } catch (UnknownHostException e) {
            System.err.println("Failed to find the specified host: "+ e.getMessage());
            System.exit(1);
        }

        try {
            networkManager.startServer();
        } catch (Manager.AlreadyStarted alreadyStarted) {
            alreadyStarted.printStackTrace();
            System.exit(1);
        } catch (SocketException e) {
            e.printStackTrace();
            System.exit(1);
        }

        //  Create a ControllerState and a ControllerUpdater object to send updates of the DPAD buttons to the rover
        try {
            //BaseStation base = new BaseStation();
            ControllerState controllerChanging = new ControllerState();
            ControllerUpdater sendDpad = new ControllerUpdater(networkManager);
            controllerChanging.addObserver(sendDpad);

        }
        catch (Exception e){
            System.out.println("Failed to observe things: " + e.getMessage());

        }

        Application.launch(DisplayApplication.class);
    }
}
