import socket
import time
import sys

args = sys.argv[1:]
if len(args) < 2:
    print("[!] Usage: python udp_client.py <IP Address> <port>")
    exit(1)

# IP Address for server received as input
cs_UDP_IP_ADDRESS = args[0]
# Port for the server entered on the command line as well
cn_UDP_PORT_NO = int(args[1])

# Arbitrary message to send to server
cs_message = "Hello, Server"

# Tries to create a socket and prints a message if it fails
try:
    clientSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
except socket.error:
    print ('Failed to create socket')
    sys.exit()


# Counts how many times a message has been sent
cn_count = 0

# Infinite loop that sends a message every 3 seconds to the server with the given IP Address and port number
while True:
    clientSock.sendto(cs_message.encode("utf-8"), (cs_UDP_IP_ADDRESS, cn_UDP_PORT_NO))
    cn_count = cn_count + 1
    if(cn_count == 1):
        print("Message sent " + str(cn_count) + " time :)")
    else:
        print("Message sent " + str(cn_count) + " times :)")
    time.sleep(3)
