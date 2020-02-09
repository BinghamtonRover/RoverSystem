SOURCES = main.cpp mutils.cpp program.cpp grid_program.cpp model.cpp fill_program.cpp rover_model.cpp rover_program.cpp obstacle.cpp map.cpp lidar.cpp lidar_point_model.cpp cell_model.cpp 
CFLAGS = -std=c++14 -Wall -g -pthread
LFLAGS = `pkg-config --libs glfw3` -ldl -lXinerama -lXxf86vm -lXcursor -lXrandr  -lm -lGL -lGLEW -lX11 -L../../bin -ltoml -lautonomy
OUT = ../../bin/simulator

SOURCES += $(AUTONOMY_SOURCE)

$(OUT): $(SOURCES)
	g++ -o $@ $(CFLAGS) $(SOURCES) $(LFLAGS)

.PHONY: run
run: $(OUT)
	$(OUT)
