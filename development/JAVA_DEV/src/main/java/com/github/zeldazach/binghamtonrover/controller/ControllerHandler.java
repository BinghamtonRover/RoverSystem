package com.github.zeldazach.binghamtonrover.controller;

import net.java.games.input.Controller;
import net.java.games.input.Event;
import net.java.games.input.EventQueue;

public class ControllerHandler implements Runnable {

    private Controller controller;
    private ControllerState state;

    public ControllerHandler(Controller controller) {
        this.controller = controller;
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
                state.update(event.getComponent().getName(), event.getValue());
            }
        }
    }
}
