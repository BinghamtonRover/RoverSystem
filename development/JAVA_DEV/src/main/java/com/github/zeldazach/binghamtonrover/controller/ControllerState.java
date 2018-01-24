package com.github.zeldazach.binghamtonrover.controller;

import java.util.Observable;

/**
 * ControllerState represents the state of the XBox controller at a given moment in time.
 */
public class ControllerState extends Observable {

    /**
     * The margin about a floating-point number in which other floats are considered to be equal.
     */
    public static final float EQUALITY_MARGIN = 1.0E-6f;

    /**
     * The threshold about 0.0 at which a stick movement will actually be processed.
     * This is fixed for now, but could be calibrated at runtime if necessary.
     */
    public static final float STICK_THRESHOLD = 0.02f;

    /**
     * Simple buttons, have an off state (false) and an on state (true).
     *
     * Select is the left-most middle button, mode is the glowing button with the logo, and start is the right-most middle button.
     * LThumb and RThumb are the left and right sticks when pressed.
     */
    public boolean buttonX, buttonY, buttonA, buttonB, buttonSelect, buttonMode, buttonStart, buttonLBumper, buttonRBumper, buttonLThumb, buttonRThumb;

    /**
     * Normalized values (between -1 and 1) for "continuous" axes.
     *
     * lTrigger and rTrigger are from -1 (not pressed) to 1 (fully pressed).
     *
     * The dpad values are interesting. They rage from 0 (meaning none pressed) to 1 (meaning left pressed).
     * Up is 0.25, right is 0.5, down is 0.75, and left is 1.0. If two adjacent are pressed together, the values is
     * between those listed above. Chart:
     *
     * Left + Up     : 0.125
     * Up            : 0.25
     * Up + Right    : 0.375
     * Right         : 0.5
     * Right + Down  : 0.675
     * Down          : 0.75
     * Down + Left   : 0.875
     * Left          : 1.0
     */
    public float lStickX, lStickY, rStickX, rStickY, lTrigger, rTrigger, dpad;

    public void update(String name, float value) {
        switch (name) {
            case "X":
                buttonX = float_to_bool(value);
                break;
            case "Y":
                buttonY = float_to_bool(value);
                break;
            case "A":
                buttonA = float_to_bool(value);
                break;
            case "B":
                buttonB = float_to_bool(value);
                break;
            case "Select":
                buttonSelect = float_to_bool(value);
                break;
            case "Mode":
                buttonMode = float_to_bool(value);
                break;
            case "Start":
                buttonStart = float_to_bool(value);
                break;
            case "Left Thumb":
                buttonLBumper = float_to_bool(value);
                break;
            case "Right Thumb":
                buttonRBumper = float_to_bool(value);
                break;
            case "Left Thumb 3":
                buttonLThumb = float_to_bool(value);
                break;
            case "Right Thumb 3":
                buttonRThumb = float_to_bool(value);
                break;
            case "x":
                lStickX = clamp_stick_value(value);
                break;
            case "y":
                lStickY = clamp_stick_value(value);
                break;
            case "rx":
                rStickX = clamp_stick_value(value);
                break;
            case "ry":
                rStickY = clamp_stick_value(value);
                break;
            case "z":
                lTrigger = value;
                break;
            case "rz":
                rTrigger = value;
                break;
            case "pov":
                dpad = value;
                break;
            default:
                throw new RuntimeException("ControllerState received update with unknown button name \"" + name + "\"");
        }

        setChanged();
        notifyObservers();
    }

    private boolean float_to_bool(float value) {
        if (Math.abs(value - 0.0f) < EQUALITY_MARGIN) {
            return false;
        }

        return true;
    }

    private float clamp_stick_value(float value) {
        if (Math.abs(value) < STICK_THRESHOLD) {
            return 0;
        }

        return value;
    }
}
