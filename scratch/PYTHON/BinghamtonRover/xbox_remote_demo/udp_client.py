"""
This is the UDP client
It will continuously send UDP packets containing the information from ControllerState to the server.
The computer running the client is the computer connected to the Xbox controller.
"""

import socket
import time
import sys
import threading
from luz_xbox import listen_for_events
from controller_state import ControllerState

# Checks if all of the arguments were entered on the command line
args = sys.argv[1:]
if len(args) < 2:
    # Prints this message if the IP Address and/or the port were not given
    print("[!] Usage: python udp_client.py <IP Address> <port>")
    exit(1)

# IP Address and port number for the server received as input on the command line
gs_UDP_IP_ADDRESS = args[0]
gn_UDP_PORT_NO = int(args[1])

# Create a ControllerState object to keep track of the state of the buttons on the remote
go_controller = ControllerState()


# Call Luz's function in another thread so that can do 2 things at once
go_thread_the_needle = threading.Thread(group=None, target=listen_for_events, args=(go_controller,), daemon=True)
go_thread_the_needle.start()

def controller_state_to_buffer(go_controller):
    # Create bytearray to send to server with buttons in the same order as in controller_state
    go_buffer = bytearray()
    # A, B, X, and Y buttons as 1 byte integer
    go_buffer.extend(go_controller.cn_a.to_bytes(1, "big"))
    go_buffer.extend(go_controller.cn_b.to_bytes(1, "big"))
    go_buffer.extend(go_controller.cn_x.to_bytes(1, "big"))
    go_buffer.extend(go_controller.cn_y.to_bytes(1, "big"))

    # Left and right bumpers as 1 byte integer
    go_buffer.extend(go_controller.cn_left_bumper.to_bytes(1, "big"))
    go_buffer.extend(go_controller.cn_right_bumper.to_bytes(1, "big"))

    # Middle, Back, and Start buttons as 1 byte integer
    go_buffer.extend(go_controller.cn_middle.to_bytes(1, "big"))
    go_buffer.extend(go_controller.cn_back.to_bytes(1, "big"))
    go_buffer.extend(go_controller.cn_start.to_bytes(1, "big"))

    # Left joystick x and y axes as a 4 byte signed integer
    go_buffer.extend(go_controller.cn_left_stick_x.to_bytes(4, "big", signed=True))
    go_buffer.extend(go_controller.cn_left_stick_y.to_bytes(4, "big", signed=True))

    # Right joystick x and y axes as a 4 byte signed integer
    go_buffer.extend(go_controller.cn_right_stick_x.to_bytes(4, "big", signed=True))
    go_buffer.extend(go_controller.cn_right_stick_y.to_bytes(4, "big", signed=True))

    # Left and Right trigger as a 1 byte integer
    lo_left_trigger = int((go_controller.cn_left_trigger/1024) * 256)
    lo_right_trigger = int((go_controller.cn_right_trigger/1024) * 256)

    go_buffer.extend(lo_left_trigger.to_bytes(1, "big"))
    go_buffer.extend(lo_right_trigger.to_bytes(1, "big"))

    # Up, Down, Left, and Right on the D-Pad as 1 byte integers
    go_buffer.extend(go_controller.cn_dpad_up.to_bytes(1, "big"))
    go_buffer.extend(go_controller.cn_dpad_down.to_bytes(1, "big"))
    go_buffer.extend(go_controller.cn_dpad_left.to_bytes(1, "big"))
    go_buffer.extend(go_controller.cn_dpad_right.to_bytes(1, "big"))


    return go_buffer


# Tries to create a socket and prints a message if it fails
try:
    go_clientSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
except socket.error:
    print ('Failed to create socket')
    sys.exit()


# Counts how many times a message has been sent
gn_count = 0

# Infinite loop that sends a message every 3 seconds to the server with the given IP Address and port number
while True:
    time.sleep(.033)
    go_clientSock.sendto(controller_state_to_buffer(go_controller), (gs_UDP_IP_ADDRESS, gn_UDP_PORT_NO))
    gn_count = gn_count + 1
    # Prints this every time the message successfully sends (debugging purposes)
    if(gn_count == 1):
        print("Message sent " + str(gn_count) + " time :)")
    else:
        print("Message sent " + str(gn_count) + " times :)")