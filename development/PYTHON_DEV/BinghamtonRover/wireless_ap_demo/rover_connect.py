"""
This file is a script that makes the rover automatically connect to the base station over wifi
"""
from .wap import _nmcli, _get_wireless_device


def is_in_range(network_name, fields, *arguments):
    """
    This looks at the list of all of the possible wifi connections for the device

    Arguments:
        network_name - The name of the network of the base station we want to connect to
        fields - A list of names of nmcli fields to collect in the response. In this case only "SSID" is passed in
        arguments - A varargs list of arguments to pass to wap._nmcli.

    Returns True if network_name is on the list and False if network_name is not on the list.
    """
    wifi_str = _nmcli(fields, *arguments)
    for line in wifi_str.splitlines():
        # Checks if the line from the list is our base stations' network
        if line.startswith(network_name):
            # The base station was found.
            print("Found the base station and now connecting...")
            return True
    print("Can't find " + network_name)
    return False


def delete():
    """
    This deletes the BUROVERDEMO wifi connection to the base station once we are done with it. This will prevent the network name
        from getting numbers added to the end of it
    """
    #Deletes the network
    _nmcli([], "c", "delete", "BUROVERDEMO")


def connect(network_name, network_password):
    """
    This connects the device to the base station through a wifi connection

    Arguments:
        network_name - The name of the base station wifi connection to connect to
        network_password - The password to the wifi of the base station. Necessary to connect to the base station
    """

    # Get the name of the default wireless device.
    device_name = _get_wireless_device()

    # Get  info on if the device is connected to a network and if so, the type and name of the connection
    connection_str = _nmcli(["DEVICE", "STATE", "CONNECTION"], "device", "status")

    # Get info on the name of the device, its connection status, and the network it is connected to, if any
    # and compares that to our desired settings from the parameters
    for line in connection_str.splitlines():
        current_device = line.split(":")[0]
        current_state = line.split(":")[1]
        current_connection = line.split(":")[2]

        # Print the device status (for debugging purposes)
        print("device " + current_device)
        print("state " + current_state)
        print("connection " + current_connection)
        print()

        # If the device is not already connected to the network from the parameter then it will:
        # Check that we are working with the correct device
        # Check if it is in range and connect  if it is
        # Otherwise it will give an exception
        if current_connection != network_name and device_name == current_device:
            if is_in_range(network_name, ["SSID"], "device", "wifi", "list"):
                _nmcli([], "dev", "wifi", "con", network_name, "password", network_password)

            else:
                raise Exception("Base station is out of range")
        # Added this if the rover recognizes multiple devices but only one of them really has to connect
        elif device_name == current_device:
            print("Already connected to the network :)")
        print()

