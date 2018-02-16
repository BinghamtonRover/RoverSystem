package com.github.zeldazach.binghamtonrover.controller;

import java.util.Observable;

/**
 * ControllerState represents the state of the XBox controller at a given moment in time.
 */
public class ControllerState extends Observable
{

    /**
     * The margin about a floating-point number in which other floats are considered to be equal.
     */
    private static final float EQUALITY_MARGIN = 1.0E-6f;

    /**
     * The threshold about 0.0 at which a stick movement will actually be processed.
     * This is fixed for now, but could be calibrated at runtime if necessary.
     */
    private static final float STICK_THRESHOLD = 0.02f;

    /**
     * Simple buttons, have an off state (false) and an on state (true).
     *
     * Select is the left-most middle button, mode is the glowing button with the logo, and start is the right-most middle button.
     * LThumb and RThumb are the left and right sticks when pressed.
     *
     * TODO: Getters/Setters possibly?
     */
    public boolean buttonX, buttonY, buttonA, buttonB, buttonSelect, buttonMode, buttonStart;
    public boolean buttonLBumper, buttonRBumper, buttonLThumb, buttonRThumb;

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

    public ControllerState() {
        // Set some defaults. We want everything to be off by default.

        lTrigger = -1.0f;
        rTrigger = -1.0f;
    }

    public void update(String name, float value)
    {
        switch (name)
        {
            case "X":
                buttonX = floatToBool(value);
                break;
            case "Y":
                buttonY = floatToBool(value);
                break;
            case "A":
                buttonA = floatToBool(value);
                break;
            case "B":
                buttonB = floatToBool(value);
                break;
            case "Select":
                buttonSelect = floatToBool(value);
                break;
            case "Mode":
                buttonMode = floatToBool(value);
                break;
            case "Start":
                buttonStart = floatToBool(value);
                break;
            case "Left Thumb":
                buttonLBumper = floatToBool(value);
                break;
            case "Right Thumb":
                buttonRBumper = floatToBool(value);
                break;
            case "Left Thumb 3":
                buttonLThumb = floatToBool(value);
                break;
            case "Right Thumb 3":
                buttonRThumb = floatToBool(value);
                break;
            case "x":
                lStickX = clampStickValue(value);
                break;
            case "y":
                lStickY = clampStickValue(value);
                break;
            case "rx":
                rStickX = clampStickValue(value);
                break;
            case "ry":
                rStickY = clampStickValue(value);
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

    private boolean floatToBool(float value)
    {
        return !(Math.abs(value - 0.0f) < EQUALITY_MARGIN);
    }

    private float clampStickValue(float value)
    {
        if (Math.abs(value) < STICK_THRESHOLD)
        {
            return 0;
        }

        return value;
    }
}