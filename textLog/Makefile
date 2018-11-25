SOURCES = textLog.cpp
CFLAGS = -std=c++14 -Wall -g `sdl2-config --cflags`
LFLAGS = `sdl2-config --libs` -lGL
OUT = textLog

$(OUT): $(SOURCES)
	g++ -o $@ $(CFLAGS) $(SOURCES) $(LFLAGS)
