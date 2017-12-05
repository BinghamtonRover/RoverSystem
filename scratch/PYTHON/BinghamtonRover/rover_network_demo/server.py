"""
This is the TCP Server
The server will take a port and a ControllerState as a parameter and connect with one client at a time
It will send the updated ControllerState to the client in a thread and receive JSON data in packets
The JSON packets will be written to '/dev/null' for now
In the future the JSON data will be written to a pipe that Java will read from
"""

import socket
import threading
import json

gs_bind_ip = '0.0.0.0'


def set_up_server(an_port, ao_controller):
    # Create the socket and start listening for clients to connect to
    lo_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    lo_server.bind((gs_bind_ip, an_port))
    # Start listening and don't keep a backlog of unaccepted connections once it connects to a client
    lo_server.listen(0)

    print ('Listening on {}:{}'.format(gs_bind_ip, an_port))

    # Connect to the client which will be the rover
    lo_client_connection, lao_client_address = lo_server.accept()
    print ('Accepted connection from {}:{}'.format(lao_client_address[0], lao_client_address[1]))

    # Create thread to send the ControllerState to the client
    lo_client_handler = threading.Thread(
        target=send_controllerState_updates,
        args=(lo_client_connection, ao_controller)
    )
    lo_client_handler.start()

    while True:
        try:
            # Receive a JSON from the rover and write it to the special file '/dev/null'
            # In the future this data will be sent to a pipe that Java reads from
            ls_request = lo_client_connection.recv(1024)
            print ('Received {}'.format(ls_request))
            with open('/dev/null', 'w') as f:
                json.dump(ls_request, f)
        finally:
            break
    # Close the socket to clean up the connection
    lo_client_connection.close()


def send_controllerState_updates(ao_client_sock, ao_controller):
    # Send the ControllerState to the rover
    # Send returns the number of bytes sent
    # Could use sendall instead but it is less reliable and doesn't tell you how much sent when an error occurs
    ao_client_sock.send(ao_controller.encode) # need to figure out how I am sending the ControllerState updates



