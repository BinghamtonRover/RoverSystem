from inputs import get_gamepad
import json

button_mappings = {
    "BTN_SOUTH": "A",
    "BTN_EAST": "B",
    "BTN_NORTH": "X",
    "BTN_WEST": "Y",
    "BTN_MODE": "XBOX",
    "BTN_SELECT": "BACK",
    "BTN_START": "START",
    "ABS_X": "X-LEFT-STICK",
    "ABS_Y": "y-left-STICK",
    "ABS_RX": "X-RIGHT-STICK",
    "ABS_RY": "Y-RIGHT-STICK",
    "ABS_Z": "LEFT TRIGGER",
    "ABS_RZ": "RIGHT TRIGGER",
    "ABS_HAT0X": "X-DPAD",
    "ABS_HAT0Y": "Y-DPAD"

}


def write_to_json():
    if event.code in button_mappings:
        json_object = {button_mappings[event.code]: event.state}

        with open('button_mappings.json', 'a') as outfile:
            json.dump(json_object, outfile, indent=4)

        print(event.code, event.state)


with open('button_mappings.json', 'w') as outfile:
    json.dump(None, outfile)
while True:
    events = get_gamepad()
    for event in events:
        write_to_json()

        # if event.code == "BTN_SOUTH" and event.state == 1:
        #     print("A")
        # elif event.code == "BTN_EAST" and event.state == 1:
        #     print("B")
        # elif event.code == "BTN_NORTH" and event.state == 1:
        #     print("x")
        # elif event.code == "BTN_WEST" and event.state == 1:
        #     print("Y")
        # elif event.code == "BTN_TL" and event.state == 1:
        #     print("Left Bumper")
        # elif event.code == "BTN_TR" and event.state == 1:
        #     print("Right Bumper")
        # elif event.code == "BTN_MODE" and event.state == 1:
        #     print("Middle Button")
        # elif event.code == "BTN_SELECT" and event.state == 1:
        #     print("Back Button")
        # elif event.code == "BTN_START" and event.state == 1:
        #     print("Start Button")
        # elif event.code == "ABS_X" and event.state < -32000:
        #     print("left on left Stick")
        # elif event.code == "ABS_Y" and event.state < -32000:
        #     print("up on left Stick")
        # elif event.code == "ABS_X" and event.state > 32000:
        #     print("right on left Stick")
        # elif event.code == "ABS_Y" and event.state > 32000:
        #     print("down on left Stick")
        # elif event.code == "ABS_RX" and event.state < -32000:
        #     print("left on right Stick")
        # elif event.code == "ABS_RY" and event.state < -32000:
        #     print("up on right Stick")
        # elif event.code == "ABS_RX" and event.state > 32000:
        #     print("right on right Stick")
        # elif event.code == "ABS_RY" and event.state > 32000:
        #     print("down on right Stick")
        # elif event.code == "ABS_Z" and event.state == 255:
        #     print("Left Trigger")
        # elif event.code == "ABS_RZ" and event.state == 255:
        #     print("Right Trigger")
        # elif event.code == "ABS_HAT0X"  and event.state == -1:
        #     print("left on D-Pad")
        # elif event.code == "ABS_HAT0X"  and event.state == 1:
        #     print("right on D-Pad")
        # elif event.code == "ABS_HAT0Y"  and event.state == -1:
        #     print("up on D-Pad")
        # elif event.code == "ABS_HAT0Y"  and event.state == 1:
        #     print("down on D-Pad")
        # #else:
        #     print([event.code, event.state])
        #

