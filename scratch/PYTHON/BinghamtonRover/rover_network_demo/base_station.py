""""
The base_station receives updates from the xbox controller...
 and sets up a TCP server to receive updates

"""
import socket
import threading
import os
import sys
import controller_display
from controller_state import ControllerState
import server
from xbox_input import listen_for_events

go_cs = controller_state()

#Thread to update controller state from xbox_inputs
go_xbox_input = threading.Thread(group=None, target=listen_for_events, args=(go_cs,), daemon=True)
go_xbox_input.start()

#IP and Port to connect to server
g_TCP_IP = ''
g_TCP_PORT = ''

#Function to start server to send xbox updates
def read_to_server()
    TCP_Server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    TCP_Server.connect((g_TCP_IP, g_TCP_PORT))
    while True:
        try:
            print "Socket successfully created"
            TCP_Server.send(go_cs)
        except socket.error:
            print('Failed to create socket')
            sys.exit()
        finally:
            break

#Thread to send xbox updates to server
go_TCP_Server = threading.Thread(group=None, target=read_to_server(), daemon=True)
go_TCP_Server.start()

#Calls controller display with Controller State Object
controller_display.start(go_cs)

