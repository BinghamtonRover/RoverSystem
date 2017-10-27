import bluetooth
import sys

#Try to find a nearby device, if not found exit
try:
    print("Searching for nearby devices...")
    nearby_devices = bluetooth.discover_devices(lookup_names=True)
except OSError:
    print("Error: Devices not found")
    sys.exit()

#Place holder, needs to be set to the client port
port = 3

for address, name in nearby_devices:
    bd_addr = address

#We found a device print its address
print("Device found! Address: " + bd_addr)

#Create a way to connect between two devices with RFCOMM protocol
socket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)

#Try to connect to the device, if device can not be connected to exit
try:
    socket.connect((bd_addr, port))
except OSError:
    print("Error: Destination host device down")
    sys.exit()
#Send a message to the device
socket.send("Hello World")

#Close connection to device
sock.close()
