from inputs import get_gamepad


while True:                                                                     #infinite loop
    events = get_gamepad()                                                      #"""Get a single action from a gamepad."""
    for event in events:                                                        # events is an object/list(I'm not sure) that holds the type of event.
        if event.code == "BTN_SOUTH" and event.state == 1:                      #the name of the button pressed, and the state of the button when pressed
            print("A")                                                          # A button
        elif event.code == "BTN_EAST" and event.state == 1:
            print("B")                                                          #B button and so on
        elif event.code == "BTN_NORTH" and event.state == 1:
            print("x")
        elif event.code == "BTN_WEST" and event.state == 1:
            print("Y")
        elif event.code == "BTN_TL" and event.state == 1:
            print("Left Bumper")
        elif event.code == "BTN_TR" and event.state == 1:
            print("Right Bumper")
        elif event.code == "BTN_MODE" and event.state == 1:
            print("Middle Button")
        elif event.code == "BTN_SELECT" and event.state == 1:
            print("Back Button")
        elif event.code == "BTN_START" and event.state == 1:
            print("Start Button")
        elif event.code == "ABS_X" and event.state < -32000:
            print("left on left Stick")
        elif event.code == "ABS_Y" and event.state < -32000:
            print("up on left Stick")
        elif event.code == "ABS_X" and event.state > 32000:
            print("right on left Stick")
        elif event.code == "ABS_Y" and event.state > 32000:
            print("down on left Stick")
        elif event.code == "ABS_RX" and event.state < -32000:
            print("left on right Stick")
        elif event.code == "ABS_RY" and event.state < -32000:
            print("up on right Stick")
        elif event.code == "ABS_RX" and event.state > 32000:
            print("right on right Stick")
        elif event.code == "ABS_RY" and event.state > 32000:
            print("down on right Stick")
        elif event.code == "ABS_Z" and event.state == 255:
            print("Left Trigger")
        elif event.code == "ABS_RZ" and event.state == 255:
            print("Right Trigger")
        elif event.code == "ABS_HAT0X"  and event.state == -1:
            print("left on D-Pad")
        elif event.code == "ABS_HAT0X"  and event.state == 1:
            print("right on D-Pad")
        elif event.code == "ABS_HAT0Y"  and event.state == -1:
            print("up on D-Pad")
        elif event.code == "ABS_HAT0Y"  and event.state == 1:
            print("down on D-Pad")
        #else:                                                                  #enable this to see the name of the button pressed and the state of the button
        #    print([event.code, event.state])
