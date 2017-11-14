"""Bring together all implementations"""
from threading import Thread

from . import (
    send_socket,
    serialize_xbox,
    rewrite_json,
    update_json,
)
from ..wireless_ap_demo.wap import access_point_manager, stop
import argparse


def start_server(port):
    # make the server for
    server = send_socket.make_server(port)
    server_thread = Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    return server, server_thread


def main(network_ssid, network_passwd, port, json_file):
    server = server_thread = None
    try:
        with access_point_manager(network_ssid, network_passwd):
            server, server_thread = start_server(port)

            while 1: #${event}:
                instruction = serialize_xbox.get_instruction()
                rewrite_json.instructions.append(instruction)
                server.queue_instruction(instruction)
                update_json.update(json_file, instruction)
    except KeyboardInterrupt:
        if server_thread is not None:
            server.shutdown()
            server_thread.join()
        stop()


if __name__ == '__main__':
    pass # command line parsing using argparse
    #Creates a parser to look at what's passed in to the CL
    parser = argparse.ArgumentParser()
    #Add argument that will be passed in
    #Help is an extra message to guide users and explain variables
    parser.add_argument("ssid", help="Network SSID")
    parser.add_argument("password", help="Network password")
    #Parse the argument as an int so that we can pass directly to main()
    parser.add_argument("port", help="Port to connect to", type=int)
    parser.add_argument("path", help="File path to pipe")
    #Read in arguments
    args = parser.parse_args()
    main(args.ssid, args.password, args.port, args.path)
