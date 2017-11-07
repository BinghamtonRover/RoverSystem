"""This module provides an API for managing a wireless access point.
It requires a Linux OS with NetworkManager
(https://wiki.gnome.org/Projects/NetworkManager) installed and in use.

This script will use the default wireless interface to host the access
point, and thus wireless internet cannot be used while the access point
is enabled. However, this script will restore a wireless connection upon
closing the access point if such a connection was in use prior to starting
the access point.
"""
import subprocess


# access_point_name is the connection name given to the access point network.
access_point_name = "binghamtonuniversityroveraccesspoint"


# previous_network_uuid holds the nmcli uuid of the wireless connection in use before activating the access point.
# If there is no such connection or if start has not yet been called, previous_network_uuid will hold the empty string.
previous_network_uuid = ""


def _nmcli(fields, *arguments):
    """A helper function for calling the nmcli command.

    :param fields: A list of names of nmcli fields to collect in the response.
        If this is empty, terse mode will be disabled.
    :param arguments: A varargs list of arguments to pass to nmcli.

    :return the string of text output by the command
    :rtype str
    """

    # -c determines whether the output is colored,
    # and no indicates that it will not be.
    if not fields:
        # Non-terse mode.
        command = ["nmcli", "-c", "no"] + list(arguments)
    else:
        # nmcli is the command.
        # -t produces terse (easily-parseable) output.
        # -f is specified before a comma-separated list
        # of fields to include in the output.
        command = ["nmcli", "-t", "-c", "no", "-f", ",".join(fields)] + list(arguments)

    result = subprocess.run(command, stdout=subprocess.PIPE)
    
    # Return the contents of stdout, as a UTF-8 encoded string.
    return result.stdout.decode("UTF-8")


def start(ssid, password):
    """Starts the wireless access point given a specified SSID and password.
    The default wireless interface will be used.

    If the default wireless interface is currently connected to a network,
    the nmcli uuid of that network will be saved to previous_network_uuid
    and the network will be disconnected.

    :param ssid: The SSID string of the access point.
    :param password: The password of the access point.
    """

    # Get a list of all networking devices (network interfaces).
    devices_str = _nmcli(["GENERAL.DEVICE", "GENERAL.TYPE"], "device", "show")

    chosen_device = None

    for line in devices_str.splitlines():
        if line.startswith("GENERAL.DEVICE:"):
            # A device was found. Remember its name.
            current_device = line.split(":")[1]
        elif line.startswith("GENERAL.TYPE:"):
            # What is the type of current_device?
            current_type = line.split(":")[1]
            if current_type == "wifi":
                # Choose it.
                chosen_device = current_device
                break
    
    if not chosen_device:
        raise RuntimeError("No wireless device found.")
    
    # List existing connections.
    active_connections_str = _nmcli(
        ["DEVICE,UUID"], "connection", "show", "--active")
    

    global previous_network_uuid
    # Check each line for our chosen device.
    for line in active_connections_str.splitlines():
        if line.startswith(chosen_device):
            # Our device is already active.
            # Save its previous connection's UUID.
            previous_network_uuid = line.split(":")[1]
            break
    
    # Start the access point.
    _nmcli([], "device", "wifi", "hotspot", "ifname", chosen_device, "con-name",
           access_point_name, "ssid", ssid, "password", password)


def stop():
    """Stop the access point.

    Also restores the previous wireless connection, if it existed.
    """

    # Delete (and stop) the access point.
    _nmcli([], "connection", "delete", access_point_name)
    
    global previous_network_uuid
    if previous_network_uuid:
        # There was a previous network. Raise it.
        _nmcli([], "connection", "up", previous_network_uuid)
        # Clear the previous network uuid.
        previous_network_uuid = ""
