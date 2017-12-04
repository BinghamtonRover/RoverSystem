import socket
import threading
import json

bind_ip = '0.0.0.0'


def set_up_server(gn_port, go_controller):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((bind_ip, gn_port))
    server.listen(5)  # max backlog of connections

    print ('Listening on {}:{}'.format(bind_ip, gn_port))

    while True:
        client_sock, client_address = server.accept()
        print ('Accepted connection from {}:{}'.format(client_address[0], client_address[1]))
        client_handler = threading.Thread(
            target=send_controllerState_updates,
            args=(client_sock, go_controller)
        )
        client_handler.start()

        try:
            # Receive a JSON from the rover and write it to the special file '/dev/null'
            # In the future this data will be sent to a pipe that Java reads from
            ls_request = client_sock.recv(1024)
            print ('Received {}'.format(ls_request))
            with open ('/dev/null', 'w') as f:
                json.dump(ls_request, f)
        finally:
            client_sock.close()


def send_controllerState_updates(client_sock, controller):
    # Send the ControllerState to the rover
    # Returns the number of bytes sent
    # Could use sendall instead but it is less reliable and doesn't tell you how much sent when an error occurs
    client_sock.send(controller)



