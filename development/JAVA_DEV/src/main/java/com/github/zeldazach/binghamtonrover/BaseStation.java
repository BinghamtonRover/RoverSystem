package com.github.zeldazach.binghamtonrover;

import com.github.zeldazach.binghamtonrover.controller.ControllerHandler;
import com.github.zeldazach.binghamtonrover.gui.DisplayApplication;
import javafx.application.Application;

public class BaseStation {

    /**
     * The entry point for the base station control program.
     * @param args Command-line arguments.
     */
    public static void main(String[] args) {
        ControllerHandler.init();

        Application.launch(DisplayApplication.class);
    }
}
