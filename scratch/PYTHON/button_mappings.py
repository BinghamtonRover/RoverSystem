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
    "ABS_Y": "Y-LEFT-STICK",
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
