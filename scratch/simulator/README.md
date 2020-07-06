**Simulator needs to be run on linux; no faking it with virtual machines or ubuntu for windows. Real linux.**

Clone the repo:
```
git clone https://github.com/BinghamtonRover/RoverSystem.git
```
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

Run make:
```
make
```
Change directories to the simulator folder:
```
cd scratch/simulator
```
Make a top level bin folder:
```
mkdir bin
```
Run make:
```
make
```
Run the simulator:
```
../../bin/simulator test.toml
```
To move around the simulator, either scroll in or out; or click and drag to move the screen.

To edit the obstacles, edit the coordinates in test.toml.

To quit, either press escape or x-out.

Whenver you make edits, run `make` and then run the simulator to view changes.