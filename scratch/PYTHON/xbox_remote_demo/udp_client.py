import socket
import time
import sys

# IP Address for server recieved as input
cs_UDP_IP_ADDRESS = input("Enter IP Address of the server: ")  # 149.125.78.78
# Port for the server entered on the command line as well
cn_UDP_PORT_NO = int(input("Enter the port number for the server: "))  # 1300
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
