SOURCES = main.cpp controller.cpp camera_feed.cpp
HEADERS = controller.hpp camera_feed.hpp
LIBRARIES = ../../bin/libnetwork.a
CFLAGS = -std=c++14 -Wall -g `sdl2-config --cflags` -I/opt/libjpeg-turbo/include
LFLAGS = `sdl2-config --libs` -lGL -L/opt/libjpeg-turbo/lib64 -l:libturbojpeg.a -L../../bin -lnetwork
OUT = ../../bin/basestation

$(OUT): $(SOURCES) $(HEADERS) $(LIBRARIES)
	g++ $(CFLAGS) $(SOURCES) -o $@ $(LFLAGS)