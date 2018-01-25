package com.github.zeldazach.binghamtonrover.controller;

import net.java.games.input.*;

import javax.swing.*;
import java.util.ArrayList;
import java.util.List;
import java.util.logging.Logger;

public class ControllerHandler implements Runnable {

    private static ControllerHandler instance;

    public static void init() {
        if (instance != null) {
            throw new IllegalStateException("ControllerHandler was already initialized!");
        }

        ControllerEnvironment loEnvironment = ControllerEnvironment.getDefaultEnvironment();

        // This prevents JInput from spamming stderr with useless warning messages.
        Logger.getLogger(ControllerEnvironment.class.getName()).setUseParentHandlers(false);

        List<Controller> laValidControllers = new ArrayList<>();

        for (Controller loController : loEnvironment.getControllers()) {
            if (loController.getType() != Controller.Type.GAMEPAD) continue;
            if (loController.getPortType() != Controller.PortType.USB) continue;

            laValidControllers.add(loController);
        }

        if (laValidControllers.isEmpty()) {
            JOptionPane.showMessageDialog(null, "No USB controllers were found!", "Error", JOptionPane.ERROR_MESSAGE);
            throw new RuntimeException("No USB controllers were found");
        }

        Controller loChosenController;

        if (laValidControllers.size() > 1) {
            System.out.println("There are more than one USB controller connected!");

            String[] laNames = new String[laValidControllers.size()];
            for (int i = 0; i < laValidControllers.size(); i++) {
                laNames[i] = laValidControllers.get(i).getName();
            }

            Controller loSelected = (Controller) JOptionPane.showInputDialog(null, "Select the controller:", "More than one USB controller was found!", JOptionPane.INFORMATION_MESSAGE, null, laValidControllers.toArray(), laValidControllers.get(0));
            if (loSelected == null) {
                throw new RuntimeException("User cancelled controller selection.");
            }

            loChosenController = loSelected;
            System.out.println("User selected controller: " + loChosenController);
        } else {
            loChosenController = laValidControllers.get(0);
            System.out.println("Defaulting to only controller: " + loChosenController);
        }

        instance = new ControllerHandler(loChosenController);

        Thread loInstanceThread = new Thread(instance);
        loInstanceThread.setDaemon(true);
        loInstanceThread.start();
    }

    public static ControllerHandler getInstance() {
        if (instance == null) {
            throw new IllegalStateException("ControllerHandler.init() MUST be called before ControllerHandler.getInstance() !!!!!");
        }

        return instance;
    }

    private Controller controller;
    private ControllerState state;

    public ControllerHandler(Controller controller) {
        this.controller = controller;
        this.state = new ControllerState();
    }

    public ControllerState getState() {
        return state;
    }

    @Override public void run() {
        while (true) {
            controller.poll();

            EventQueue queue = controller.getEventQueue();

            Event event = new Event();

            while (queue.getNextEvent(event)) {
                System.out.println("PRESS " + event.getComponent().getName());
                state.update(event.getComponent().getName(), event.getValue());
            }
//
//            for (Component loComponent : controller.getComponents()) {
//                state.update(loComponent.getName(), loComponent.getPollData());
//            }

            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {}
        }
    }
}
