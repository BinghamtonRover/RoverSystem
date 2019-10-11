# Base Station Setup and Operation

This document describes the process of configuring the base station environment.

It is assumed that the base OS is freshly installed Ubuntu 18.04.

## Library Installation

The base station uses GLFW to perform window / input management. Install as follows:

1. Download and extract the source code (found here: https://github.com/glfw/glfw/releases/download/3.3/glfw-3.3.zip).
2. From inside the extracted dir, run `cmake .` (you might need to install cmake with `sudo apt install cmake`).
3. Run `make`.
4. Run `sudo make install`.

Then install OpenGL with `sudo apt install libgl1-mesa-dev`.

You must also install TurboJPEG:
1. Download the package file from https://sourceforge.net/projects/libjpeg-turbo/files/2.0.0/libjpeg-turbo-official_2.0.0_amd64.deb/download.
2. Run `sudo dpkg -i libjpeg-turbo-official_2.0.0_amd64.deb` to install it. Verify that a directory `/opt/libjpeg-turbo` exists.

## Building

From the repository root, run `make network base_station rover` to build the system.
