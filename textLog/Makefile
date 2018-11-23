SOURCES = main.cpp
CFLAGS = -std=c++14 -Wall -g `sdl2-config --cflags`
LFLAGS = `sdl2-config --libs` -lGL
OUT = textlog

$(OUT): $(SOURCES)
	g++ -o $@ $(CFLAGS) $(SOURCES) $(LFLAGS)

clean: 
	rm $(OUT)
