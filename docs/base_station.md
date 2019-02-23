# Base Station Setup and Operation

This document describes the process of configuring the base station environment.

It is assumed that the base OS is freshly installed Ubuntu 18.04.

## Library Installation

The base station uses GLFW to perform window / input management. Install as follows:

1. Download and extract the source code.
2. From inside the source dir, run `cmake .`.
3. Run `make`.
4. Run `sudo make install`.

Then install OpenGL and Log4CPP with `sudo apt install liblog4cplus-1.1-9 libgl1-mesa-dev`.

You must also install TurboJPEG:
1. Download the package file from https://sourceforge.net/projects/libjpeg-turbo/files/2.0.0/libjpeg-turbo-official_2.0.0_amd64.deb/download.
2. Run `sudo dpkg -i libjpeg-turbo-official_2.0.0_amd64.deb` to install it. Verify that a directory `/opt/libjpeg-turbo` exists.

## Building

From the repository root, run `make network base_station rover` to build the system.
