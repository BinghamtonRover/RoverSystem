package com.github.zeldazach.binghamtonrover;

import com.github.zeldazach.binghamtonrover.controller.ControllerHandler;
import com.github.zeldazach.binghamtonrover.gui.DisplayApplication;
import com.github.zeldazach.binghamtonrover.networking.AlreadyStarted;
import com.github.zeldazach.binghamtonrover.networking.Manager;
import javafx.application.Application;

import java.net.SocketException;
import java.net.UnknownHostException;

public class BaseStation {

    /**
     * The entry point for the base station control program.
     * @param args Command-line arguments.
     */
    public static void main(String[] args) {
        ControllerHandler.init();

        Manager networkManager = null;
        try {
            networkManager = new Manager(args[1], Integer.parseInt(args[2]), 5);
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

        Application.launch(DisplayApplication.class);
    }
}
