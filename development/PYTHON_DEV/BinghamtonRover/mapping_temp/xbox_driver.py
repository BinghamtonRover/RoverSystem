"""
I wrote this in a rush because I have a quiz tomorrow. This is just a little something to show how the xbox controller
would be mapped if it was complete.

This code works for all buttons because those are easy (either 1 or 0)
This code doesn't work for the dpad, the triggers, and the joy sticks.
Those features will come soon ;)
"""

from inputs import get_gamepad
import json

from ..controller_state import ControllerState
from ..button_mappings import button_names

controller = ControllerState()
# controller is an instance

button_mappings = {
    # mappings for the buttons and joysticks
    "BTN_SOUTH": controller.cn_a,
    "BTN_EAST": controller.cn_b,
    "BTN_NORTH": controller.cn_x,
    "BTN_WEST": controller.cn_y,
    "BTN_MODE": controller.cn_middle,
    "BTN_SELECT": controller.cn_back,
    "BTN_START": controller.cn_start,
    "ABS_X": controller.cn_left_stick_x,
    "ABS_Y": controller.cn_left_stick_y,
    "ABS_RX": controller.cn_right_stick_x,
    "ABS_RY": controller.cn_right_stick_y,
    "ABS_Z": controller.cn_left_trigger,
    "ABS_RZ": controller.cn_right_trigger,
    "BTN_TL": controller.cn_left_bumper,
    "BTN_TR": controller.cn_right_bumper

}

# list of currently pressed buttons
pressed_list = []
# list of buttons that are released
released_list = list(button_names.values())


def write_to_json():  # function that writes to the json file of the name specified in the function
    json_object = {
        "pressed": pressed_list,
        "released": released_list,
    }

    with open('button_mappings.json', 'w') as outfile:
        json.dump(json_object, outfile, indent=4)


while True:
    events = get_gamepad()
    # events is a single event from the xbox controller
    for event in events:
        # has to be defined inside the infinite loops because the dictionary uses event.state as a conditional
        # I don't know if this is any less efficient
        dpad_mappings = {  # this is the dpad mappings according to the controller state

            "ABS_HAT0X": controller.cn_dpad_left if event.state == -1 else controller.cn_dpad_right,
            "ABS_HAT0Y": controller.cn_dpad_up if event.state == -1 else controller.cn_dpad_down
        }

        # we have to do 2 if statements be cause there are two dictionaries because of the way the dpad is set up in the
        # ControllerState class
        if event.code in button_mappings: # adds to pressed_list and deletes from released_list
            button_mappings[event.code] = event.state
            if button_mappings[event.code] == 1:
                pressed_list.append(button_names[event.code])
                try:
                    for button in range(len(released_list)):
                        if released_list[button] == button_names[event.code]:
                            del released_list[button]
                except:
                    # these exceptions are in place because for some reason I get index out of range error
                    # I'll how i can fix tis later but this'll have to do for now
                    pass

            if button_mappings[event.code] == 0: # vice versa
                released_list.append(button_names[event.code])
                try:
                    for button in range(len(pressed_list)):
                        if pressed_list[button] == button_names[event.code]:
                            del pressed_list[button]
                except:
                    pass

            write_to_json()  # calling write to json
        elif event.code in dpad_mappings:  # this is where is stopped because I'm pressed for time
            dpad_mappings[event.code] = 1

