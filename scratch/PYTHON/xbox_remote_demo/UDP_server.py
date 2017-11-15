
"""
UDP Server
The Server receives packets from the UDP client at a specific port...
& deserializes the data into a Controller State Object
"""

from controller_state import ControllerState
import threading
import os
import sys
from socket import *
import controller_display


cs = ControllerState()

def packets_to_server():
    args = sys.argv[1:]
    if len(args) < 1: # Prints this message if the IP Address and/or the port were not given
        print("[!] Usage: python udp_server.py <port>")
        exit(1)

    host = "" # Host IP - localized for example demo
    cn_UDP_PORT_NO = int(args[0]) # Port to connect with client
    addr = (host, cn_UDP_PORT_NO) #Socket takes parameter of host and port

    buffer = bytearray(31)

    UDPSock = socket(AF_INET, SOCK_DGRAM)

    UDPSock.bind(addr) #Socket binds to address

    print ("Waiting to receive UDP Client Packet...")

    while True:

        UDPSock.recvfrom_into(buffer)

        # A, B, X, and Y buttons. 0 is off and 1 is on.
        cs.cn_a = int.from_bytes(buffer[0:1], "big")
        cs.cn_b = int.from_bytes(buffer[1:2], "big")
        cs.cn_x = int.from_bytes(buffer[2:3], "big")
        cs.cn_y = int.from_bytes(buffer[3:4], "big")

        # Left and right bumpers. 0 is off and 1 is on.
        cs.cn_left_bumper = int.from_bytes(buffer[4:5], "big")
        cs.cn_right_bumper = int.from_bytes(buffer[5:6], "big")

        # Middle, back, and start buttons, whatever those are. 0 is off and 1 is on.
        cs.cn_middle = int.from_bytes(buffer[6:7], "big")
        cs.cn_back = int.from_bytes(buffer[7:8], "big")
        cs.cn_start = int.from_bytes(buffer[8:9], "big")

        # Left joystick x and y axes. Signed 4-byte integers.
        cs.cf_left_stick_x = int.from_bytes(buffer[9:13], "big")
        cs.cf_left_stick_y = int.from_bytes(buffer[13:17], "big")

         # Right joystick x and y axes. Signed 4-byte integers.
        cs.cf_right_stick_x = int.from_bytes(buffer[17:21], "big")
        cs.cf_right_stick_y = int.from_bytes(buffer[21:25], "big")

          # Left and right trigger, whatever those are. 0 is off and 1 is on.
        cs.cn_left_trigger = int.from_bytes(buffer[25:26], "big")
        cs.cn_right_trigger = int.from_bytes(buffer[26:27], "big")

           # Up, down, left, and right on the D-Pad. 0 is off and 1 is on.
        cs.cn_dpad_up = int.from_bytes(buffer[27:28], "big")
        cs.cn_dpad_down = int.from_bytes(buffer[28:29], "big")
        cs.cn_dpad_left = int.from_bytes(buffer[29:30], "big")
        cs.cn_dpad_down = int.from_bytes(buffer[30:31], "big")

        UDPSock.close()
        os._exit(0)

Object_thread = threading.Thread(group=none, target=packets_to_server, daemon=True)
Object_thread.start()

while True:
    controller_display(cs)
