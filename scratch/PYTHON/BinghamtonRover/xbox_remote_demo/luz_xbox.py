from inputs import get_gamepad
from controller_state import *

"""
This function takes a ControllerState object as parameter and listens to events to update the status of which buttons
are being pressed on the controller. 
This is called by udp_client which sends the controller's button updates in packets to the server.
"""
def listen_for_events(controller):

    button_mappings = {
        # mappings for the buttons and joysticks
        "BTN_SOUTH": "cn_a",
        "BTN_EAST": "cn_b",
        "BTN_NORTH": "cn_x",
        "BTN_WEST": "cn_y",
        "BTN_MODE": "cn_start",
        "BTN_SELECT": "cn_middle",
        "BTN_START": "cn_back",
        "ABS_X": "cn_left_stick_x",
        "ABS_Y": "cn_left_stick_y",
        "ABS_RX": "cn_right_stick_x",
        "ABS_RY": "cn_right_stick_y",
        "ABS_Z": "cn_left_trigger",
        "ABS_RZ": "cn_right_trigger",
        "BTN_TL": "cn_left_bumper",
        "BTN_TR": "cn_right_bumper"
    }

    while True:
        events = get_gamepad()
        #events is a single event from the xbox controller
        for event in events:
            if event.code == "ABS_Z":
                print("LEFTTRIGGER", event.state)
            if event.code == "ABS_HAT0X":
                if event.state == 1:
                    controller.cn_dpad_right = 1
                    controller.cn_dpad_left = 0
                elif event.state == -1:
                    controller.cn_dpad_left = 1
                    controller.cn_dpad_right = 0
                else:
                    controller.cn_dpad_left = 0
                    controller.cn_dpad_right = 0
            elif event.code == "ABS_HAT0Y":
                if event.state == 1:
                    controller.cn_dpad_down = 1
                    controller.cn_dpad_up = 0
                elif event.state == -1:
                    controller.cn_dpad_up = 1
                    controller.cn_dpad_down = 0
                else:
                    controller.cn_dpad_up = 0
                    controller.cn_dpad_down = 0
            elif event.code in button_mappings:
                setattr(controller, button_mappings[event.code], event.state)
