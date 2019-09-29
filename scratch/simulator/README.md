This simulator requires GLFW as well as OpenGL development files.

To install GLFW:

1. Download and extract the source code (found here: https://github.com/glfw/glfw/releases/download/3.3/glfw-3.3.zip).
2. From inside the extracted dir, run `cmake .` (you might need to install cmake with `sudo apt install cmake`).
3. Run `make`.
4. Run `sudo make install`.

Then install OpenGL with `sudo apt install libgl1-mesa-dev`.

See test.toml for an example level file.
