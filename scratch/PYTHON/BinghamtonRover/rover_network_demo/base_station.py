""""
The base_station receives updates from the xbox controller...
 and sets up a TCP server to receive updates

"""
from socket import *
import threading
import os
import sys
import controller_display
from controller_state import ControllerState
import server
from xbox_input import listen_for_events


go_cs = controller_state()

listen_for_events(go_cs)

def cs_TCP_Server()

    # TCP_Server = server.set_up_server(gn_port, go_cs)
    TCP_IP = ''
    TCP_PORT = ''

    try:
        TCP_Server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print "Socket successfully created"
    except socket.error as err:
        print "socket creation failed with error %s" %(err)

    TCP_Server.connect((TCP_IP,TCP_PORT))
    listen_for_events(go_cs)

go_thread = threading.Thread(target=cs_TCP_Server daemon=True)
go_thread.start()

controller_display.start(go_cs)
