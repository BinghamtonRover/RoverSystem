SOURCES = main.cpp mutils.cpp program.cpp grid_program.cpp model.cpp fill_program.cpp rover_model.cpp rover_program.cpp obstacle.cpp map.cpp lidar.cpp lidar_point_model.cpp occupancy_grid.cpp cell_model.cpp world.cpp
CFLAGS = -std=c++14 -Wall -g -pthread
LFLAGS = `pkg-config --libs glfw3` -ldl -lXinerama -lXxf86vm -lXcursor -lXrandr  -lm -lGL -lGLEW -lX11 -L../../bin -ltoml
OUT = ../../bin/simulator

AUTONOMY_SOURCE = autonomy.cpp

SOURCES += $(AUTONOMY_SOURCE)

$(OUT): $(SOURCES)
	g++ -o $@ $(CFLAGS) $(SOURCES) $(LFLAGS)

.PHONY: run memcheck
run: $(OUT)
	$(OUT)
