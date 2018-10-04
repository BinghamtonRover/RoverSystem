SOURCES = main.cpp
HEADERS = 
CFLAGS = -std=c++11 -Wall -g
LFLAGS = `sdl2-config --cflags --libs`
OUT = ../../bin/basestation

$(OUT): $(SOURCES) $(HEADERS)
	g++ $(CFLAGS) $(SOURCES) -o $@ $(LFLAGS)