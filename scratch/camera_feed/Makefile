SOURCES = display_camera_feed.cpp
CFLAGS = -std=c++14 -Wall -g
LFLAGS = `sdl2-config --libs` -L/opt/libjpeg-turbo/lib64 -l:libturbojpeg.a -L../../bin -lnetwork
OUT = ../../bin/sdl_camerafeed_demo

$(OUT): $(SOURCES) 
	g++ -o $@ $(CFLAGS) $(SOURCES) $(LFLAGS)

.PHONY: clean
clean:
	 rm $(OUT)
