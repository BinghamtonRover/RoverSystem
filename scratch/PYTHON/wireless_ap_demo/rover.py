"""
This script pretends to be the "rover".
Once it is connected to the "base station"'s network, it will send an image file to the base station.
That connection will be kept alive until an image is received, at which time the connection will be closed
and the rover will display the image.
"""

import socket
import sys
import tempfile

# The base station will always have this IP when using NetworkManager to create the hotspot.
BASE_STATION_IP = "10.42.0.1"


def receive_file(sock):
    """
    receive_file reads a file from the given socket and stores it in a temporary file,
    returning the path to that file.

    :param sock: The socket from which to read.
    :return: The path to the temporary file.
    """

    # Read 4 bytes from the socket and convert them to an integer.
    file_length = int.from_bytes(sock.recv(4), "big")

    print(file_length)

    # Read file_length bytes from the socket into memory.
    file_contents = bytearray()
    bytes_read = 0

    # Keep reading until we have the whole file.
    while bytes_read < file_length:
        buffer = sock.recv(2048)
        file_contents.extend(buffer)
        bytes_read += len(buffer)

    file_handle = tempfile.NamedTemporaryFile("w+b", suffix=".png", delete=False)
    file_handle.write(file_contents)
    file_name = file_handle.name

    # Close the temporary file.
    file_handle.close()

    return file_name


def main(port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((BASE_STATION_IP, port))

    temporary_file_name = receive_file(sock)

    print(temporary_file_name)


if __name__ == "__main__":
    args = sys.argv[1:]

    if len(args) < 1:
        print("[!] Usage: python rover.py <port>")
        exit(1)

    port = int(args[0])

    main(port)
