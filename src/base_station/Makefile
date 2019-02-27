SOURCES = main.cpp controller.cpp camera_feed.cpp gui.cpp debug_console.cpp log.cpp log_view.cpp math.cpp
HEADERS = controller.hpp camera_feed.hpp gui.hpp debug_console.hpp log.hpp log_view.hpp math.hpp
LIBRARIES = ../../bin/libnetwork.a
CFLAGS = -std=c++14 -Wall -g -I/opt/libjpeg-turbo/include -pthread
LFLAGS = `pkg-config --static --libs glfw3` -lGL -lX11 -ldl -L/opt/libjpeg-turbo/lib64 -l:libturbojpeg.a -L../../bin -lnetwork
OUT = ../../bin/basestation

$(OUT): $(SOURCES) $(HEADERS) $(LIBRARIES)
	g++ $(CFLAGS) $(SOURCES) -o $@ $(LFLAGS)
