# Rover Autonomy Simulator
**Simulator needs to be run on linux; no faking it with virtual machines or ubuntu for windows. Real linux.**

Clone the repo:
```
git clone https://github.com/BinghamtonRover/RoverSystem.git
```
## Installing dependencies

This simulator requires GLFW and GLEW as well as OpenGL development files.

To install GLFW:

1. Download and extract the source code (found here: https://github.com/glfw/glfw/releases/download/3.3/glfw-3.3.zip).
2. From inside the extracted dir, run `cmake .` (you might need to install cmake with `sudo apt install cmake`).
3. Run `make`.
4. Run `sudo make install`.

TO install GLEW:

1. Download and extract the source code (found here: https://sourceforge.net/projects/glew/files/glew/2.1.0/glew-2.1.0.zip/download).
2. From inside the extracted dir, run `sudo make install`.

Then install OpenGL with `sudo apt install libgl1-mesa-dev`.

See test.toml for an example level file.

## Building the simulator

From RoverSystem top level directory run make:
```
make
```
Make a top level bin folder:
```
mkdir bin
```
Change directories to the autonomy folder:
```
cd src/autonomy
```
Run make:
```
make
```
Change directories to the tomlc99 folder:
```
cd ../../scratch/simulator/tomlc99
```
Run make:
```
make
```
Change directories to the simulator folder:
```
cd ../
```
Run make:
```
make
```
Run the simulator (you don't need to be connected to robot or anything; all you need is your computer and the argument for which .toml file to point to):
```
../../bin/simulator test.toml
```
To move around the simulator, either scroll in or out; or click and drag to move the screen.

To edit the obstacles, edit the coordinates in test.toml.

Or, to edit obstacles during run time, hold left-ctrl and left-click on as many points on the simulator as you wish. Each click will serve as a vertex (minumum 3 vertices) for the new obstacle.

To quit, either press escape or x-out.

Whenever you make edits, run `make` and then run the simulator to view changes.
