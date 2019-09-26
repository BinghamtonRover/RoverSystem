SOURCES = main.c mutils.c program.c grid_program.c model.c fill_program.c rover_model.c rover_program.c obstacle.c map.c tomlc99/toml.c lidar.c lidar_point_model.c occupancy_grid.c cell_model.c world.c
CFLAGS = -std=c99 -Wall -g -pthread
LFLAGS = `pkg-config --libs glfw3` -ldl -lXinerama -lXxf86vm -lXcursor -lXrandr  -lm -lGL -lGLEW -lX11 
OUT = ../../bin/simulator

AUTONOMY_SOURCE = autonomy.c

SOURCES += $(AUTONOMY_SOURCE)

$(OUT): $(SOURCES)
	gcc -o $@ $(CFLAGS) $(SOURCES) $(LFLAGS)

.PHONY: run
run: $(OUT)
	$(OUT)
