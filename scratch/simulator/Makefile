SOURCES = main.c mutils.c program.c grid_program.c model.c hex_model.c fill_program.c rover_model.c rover_program.c obstacle.c map.c tomlc99/toml.c lidar.c lidar_point_model.c occupancy_grid.c
CFLAGS = -std=c99 -Wall -g -pthread
LFLAGS = `pkg-config --libs glfw3` -ldl -lXinerama -lXxf86vm -lXcursor -lXrandr  -lm -lGL -lGLEW -lX11 
OUT = ../../bin/simulator

$(OUT): $(SOURCES)
	gcc -o $@ $(CFLAGS) $(SOURCES) $(LFLAGS)
