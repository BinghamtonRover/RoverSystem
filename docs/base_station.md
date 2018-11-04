# Base Station Setup and Operation

This document describes the process of configuring the base station environment.

It is assumed that the base OS is freshly installed Ubuntu 18.04.

It is assumed that the x86_64 architecture is being targeted.

## Library Installation

### SDL

The base station uses SDL to perform window / input management. Install it with `sudo apt install libsdl2-dev`.

### OpenGL

The base station uses OpenGL for rendering the UI. Install the necessary libraries with `sudo apt install libgl1-mesa-dev`.

### TurboJpeg

TurboJpeg provides an API for very efficient JPEG encoding/decoding. This is currently used for the camera streams.

1. Download the package file from https://sourceforge.net/projects/libjpeg-turbo/files/2.0.0/libjpeg-turbo-official_2.0.0_amd64.deb/download.
2. Run `sudo dpkg -i libjpeg-turbo-official_2.0.0_amd64.deb` to install it. Verify that a directory `/opt/libjpeg-turbo` exists.
