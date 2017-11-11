"""Bring together all implementations"""
from threading import Thread

from . import (
    send_socket,
    serialize_xbox,
    rewrite_json,
    update_json,
)
from ..wireless_ap_demo.wap import access_point_manager, stop


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