SOURCES = display_camera_feed.cpp
HEADERS = display_camera_feed.hpp
LFLAGS = `sdl2-config --libs` -L/opt/libjpeg-turbo/lib64 -l:libturbojpeg.a -L../../bin -lnetwork
CFLAGS = -Wall -Wextra -pedantic -g -std=c++14  
OUT = SDL_Camera

$(OUT):	$(SOURCES) $(HEADERS)
	g++ -o $@ $(CFLAGS) $(SOURCES) $(LFLAGS) 

.PHONY:	clean
clean:
	 rm $(OUT)
