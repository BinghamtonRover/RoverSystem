package com.github.zeldazach.binghamtonrover.controller;

// TODO: Change this to import what you need instead of wildcard
import net.java.games.input.*;

// TODO: Change this to import what you need instead of wildcard
import javax.swing.*;

import java.util.ArrayList;
import java.util.List;
import java.util.logging.Logger;

public class ControllerHandler implements Runnable
{
    private static ControllerHandler controllerHandler;
    private Controller controller;
    private ControllerState controllerState;

    public static void init()
    {
        if (controllerHandler != null)
        {
            throw new IllegalStateException("ControllerHandler was already initialized!");
        }

        ControllerEnvironment controllerEnvironment = ControllerEnvironment.getDefaultEnvironment();

        // This prevents JInput from spamming stderr with useless warning messages.
        Logger.getLogger(ControllerEnvironment.class.getName()).setUseParentHandlers(false);

        // Holds potentially valid controllers.
        List<Controller> validControllersList = new ArrayList<>();

        for (Controller controller : controllerEnvironment.getControllers())
        {
            if (controller.getType() != Controller.Type.GAMEPAD)
            {
                continue;
            }

            validControllersList.add(controller);
        }

        if (validControllersList.isEmpty()) {
            System.out.println("> No valid controllers were found! Defaulting to keyboard input.");
        }

        // The controller chosen by the user.
        // Will remain null if no controllers exist.
        Controller chosenController = null;

        if (validControllersList.size() > 1) {
            System.out.println("There are more than one USB controller connected!");

            Controller selectedController =
                    (Controller) JOptionPane.showInputDialog(null,
                                                             "Select the controller:",
                                                             "More than one USB controller was found!",
                                                             JOptionPane.INFORMATION_MESSAGE,
                                                             null,
                                                             validControllersList.toArray(),
                                                             validControllersList.get(0));

            chosenController = selectedController;
            System.out.println("> User selected controller: " + chosenController);
        }
        else if (validControllersList.size() == 1)
        {
            chosenController = validControllersList.get(0);
            System.out.println("> Defaulting to only controller: " + chosenController);
        }

        controllerHandler = new ControllerHandler(chosenController);

        // Only start listening if we have a controller.
        if (chosenController != null) {
            // Launch thread to listen for controller input.
            Thread controllerHandlerThread = new Thread(controllerHandler);
            controllerHandlerThread.setDaemon(true);
            controllerHandlerThread.start();
        } else {
            System.out.println("> No controller found or selected. Using keyboard input only.");
        }
    }


    public static ControllerHandler getInstance()
    {
        if (controllerHandler == null)
        {
            throw new IllegalStateException("ControllerHandler.init() MUST be called before ControllerHandler.getInstance() !!!!!");
        }

        return controllerHandler;
    }

    private ControllerHandler(Controller c)
    {
        controller = c;
        controllerState = new ControllerState();
    }

    public ControllerState getControllerState()
    {
        return controllerState;
    }

    @Override
    public void run()
    {
        // Continuously polls the controller.
        while (true) {
            controller.poll();

            EventQueue queue = controller.getEventQueue();

            Event event = new Event();

            while (queue.getNextEvent(event))
            {
                controllerState.update(event.getComponent().getName(), event.getValue());
            }

            try
            {
                Thread.sleep(100);
            }
            catch (InterruptedException e)
            {
                System.exit(0);
            }
        }
    }
}