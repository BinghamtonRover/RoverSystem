package com.github.zeldazach.binghamtonrover.controller;

import javafx.event.EventHandler;
import javafx.scene.Scene;
import javafx.scene.input.KeyEvent;

public class KeyboardHandler {

    /**
     **     Called once to register all keyboard handlers.
     **/
    public static void registerHandlers(Scene scene) {
        // Create event handler for when the key is pressed and change the controllerState dpad values according to
        // the list of values defined in ControllerState
        EventHandler<KeyEvent> pressedHandler = event -> {
            switch (event.getCode()) {
                case W:
                    ControllerHandler.getInstance().getControllerState().update("pov", 0.25f);
                    break;
                case S:
                    ControllerHandler.getInstance().getControllerState().update("pov", 0.75f);
                    break;
                case A:
                    ControllerHandler.getInstance().getControllerState().update("pov", 1.0f);
                    break;
                case D:
                    ControllerHandler.getInstance().getControllerState().update("pov", 0.5f);
                    break;
                default:
                    break;
            }
        };

        // Create event handler for when the key is released and change the controllerState dpad values to 0
        // Specify each direction in case of future additions
        EventHandler<KeyEvent> releasedHandler = event -> {
            switch (event.getCode()) {
                case W:
                case S:
                case A:
                case D:
                    ControllerHandler.getInstance().getControllerState().update("pov", 0.0f);
                    break;
                default:
                    break;
            }
        };

        // Add event handlers for the two types of KeyEvents to the scene
        scene.addEventHandler(KeyEvent.KEY_PRESSED, pressedHandler);
        scene.addEventHandler(KeyEvent.KEY_RELEASED, releasedHandler);
    }
}
