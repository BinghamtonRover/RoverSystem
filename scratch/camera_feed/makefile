CC = g++
SOURCES = display_camera_feed.cpp
HEADERS = display_camera_feed.hpp
OBJECTS = display_camera_feed.o
#OUTDIR  = ../../bin/cameraFeed
#OFILE   = $(OUTDIR)/display_camera_feed.o

# Update these paths to match your installation
# You may also need to update the linker option rpath, which sets where to look for
# the SDL2 libraries at runtime to match your install
LIBS = `sdl2-config --cflags --libs` -I/usr/local/include -I/opt/libjpeg-turbo/include -L/usr/local/lib -L/usr/lib/x86_64-linux-gnu/libturbojpeg -L../../bin  -lSDL2 -lnetwork -l:libturbojpeg.a
# You may need to change -std=c++11 to -std=c++0x if your compiler is a bit older
CFLAGS = -Wall -g -std=c++14  
EXEC = SDL_Camera
LFLAGS = $(LIBS)
# Main target
$(EXEC):	$(OBJECTS) 
	$(CC) $(OBJECTS) -o $(EXEC) $(LFLAGS) 
# To obtain object files
$(OBJECTS): $(SOURCES) $(HEADERS)
	$(CC) $(CFLAGS) -c $(SOURCES)   

#all: $(EXE)

#$(EXE):	$(SOURCES) $(HEADERS)

#	$(CC) -c $(SOURCES) $(CFLAGS) $(SDL_LIB) -o $@ $(LFLAGS)

#display_camera_feed.o: display_camera_feed.cpp

#	$(CC) $(CXXFLAGS) $< -o $@
.PHONY:	clean
clean:
	 rm $(EXEC) && rm *.o
