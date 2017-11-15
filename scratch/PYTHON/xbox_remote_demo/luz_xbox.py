from inputs import get_gamepad
from controller_state import *

controller = ControllerState()
# controller is an instance

button_mappings = {
    #mappings for the buttons and joysticks
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
    "ABS_RZ": controller.cn_right_trigger

}

while True:
    events = get_gamepad()
    #events is a single event from the xbox controller
    for event in events:


        dpad_mappings = { # this is the dpad mappings according to the controller state
            "ABS_HAT0X": controller.cn_dpad_left if event.state == -1 else controller.cn_dpad_right,
            "ABS_HAT0Y": controller.cn_dpad_up if event.state == -1 else controller.cn_dpad_down
        }

        #we have to do 2 if statements be cause there are two dictionaries because of the way the dpad is set up in the
        #ControllerState class
        if event.code in button_mappings:
            button_mappings[event.code] = event.state
            print(button_mappings[event.code])
        elif event.code in dpad_mappings:
            dpad_mappings[event.code] = 1
            print(dpad_mappings[event.code])










