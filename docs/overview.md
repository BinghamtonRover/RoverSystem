# Binghamton University Mars Rover System Overview

This document describes the rover software and system architecture at a broad scale.

## Table of Contents

1. [Overview](#overview)
2. [Base Station](#base-station)
3. [Rover](#rover)

## Overview

The rover system is comprised of two main components: the base station and the rover. The base station controls the rover's actions and displays sensor and diagnostic information. The two components communicate via a network protocol described in another document. Since both components are on a local network, they have hardcoded IP addresses. Although this implies that each component knows the location of the other, it is the base station's responsibility to initiate connection to the rover, and to recover it in the event of connection loss. Since the base station is used to query rover state and to execute remote commands, it is fitting to describe the rover as a server and the base station as a client; however, since there is only one base station, there is no need to allow for multiple connections to the rover.

## Base Station

The base station is the control center for the rover. It is used to control the rover in manual mode, to view the rover's camera feeds, to view rover sensor data and logs, and to command the rover to complete autonomous tasks. The base station GUI is made using QT. Manual mode rover navigation is controlled by an XBox controller, and other commands are controlled with the keyboard. Mouse usage should be avoided because in the heat of competition (and the desert), no one wants to (a) use a laptop trackpad or (b) find a place suitable for an external mouse.

Here is a rough layout diagram of the GUI:
![Base Station GUI Rough Layout](Base%20Station%20GUI%20Rough%20Layout.png)

The camera feed is the main visual entity. It displays a single camera's feed at a time, and cameras can be switched using a keybinding. The camera feed occupies the largest amount of space of any single component.

Visual indicators of sensor data line the sides, and are necessary for certain segments of the competition. Data such as battery life is possibly included in this section. Some sensor data must be tracked over time, so either (a) a graph visualization will appear in the sensor area, or (b) a key press will open another screen with more detail.

The log component holds a text log of debug / error information from the rover. When debugging, the log displays important information about the rover's internal processes; during live execution, the log conveys details about rover failures. The log can be scrolled (one line at a time, possibly with up/down arrows, and multiple lines, possibly with ctrl + up/down arrows), but will auto-scroll by default.

Rover status indicators line the bottom center of the window. These give a quick visual indication of current rover state. Additionally, when in manual mode, it is helpful to receive feedback concerning the rover movement requested by XBox controller operation. For example, while current movement speed and turning direction can be inferred from the camera feed, it could be helpful to have visual indicators for those measurements.

A component consisting of network status indicators is located in the bottom-left corner of the window. It contains information such as the current ping between the rover and the base station, the strength of the wireless connection between them, and the protocol connection status (i.e. disconnected, troubled, or connected). These indicators have distinct color and are easily readable, since they are very important and will constantly be monitored during rover operation.

The bottom right corner of the window is reserved for text reminding the user of the keybindings. Since there might be many keybindings, this space should probably simply instruct the user of which key to press to display a help page which then lists all keybindings.

If remembering keybindings is a problem, it is a good idea to place small reminders next to components. For example, a label of "Press x to rotate feed" could appear below the camera feed. "Press x to show data over time" could appear below a sensor indicator. "Press x to scroll up, x to scroll down, and x to autoscroll" could appear below the rover log.

As previously mentioned, other (fullscreen) views for displaying data over time and keybinding help may exist. In addition, it may be helpful to include a view for the rover log (to display more lines) and a view for the camera feed (to display a larger image).

A debug console may also be helpful, and would allow the entry of commands. This way, actions may be triggered during debugging without adding any extra keybindings or GUI clutter.

## Rover