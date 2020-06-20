SOURCES = main.cpp controller.cpp camera_feed.cpp gui.cpp debug_console.cpp log_view.cpp waypoint.cpp waypoint_map.cpp shared_feeds.cpp session.cpp
HEADERS = controller.hpp camera_feed.hpp gui.hpp debug_console.hpp log_view.hpp waypoint.hpp waypoint_map.hpp shared_feeds.hpp ../util/util.hpp constant_vars.hpp session.hpp
LIBRARIES = ../../bin/libnetwork.a ../../bin/libsimpleconfig.a ../../bin/liblogger.a
CFLAGS = -std=c++14 -Wall -g -I/opt/libjpeg-turbo/include -pthread
LFLAGS = `pkg-config --static --libs glfw3` -lGL -lX11 -ldl -L/opt/libjpeg-turbo/lib64 -l:libturbojpeg.a -L../../bin -lnetwork -lsimpleconfig -llogger
OUT = ../../bin/basestation

$(OUT): $(SOURCES) $(HEADERS) $(LIBRARIES)
	g++ $(CFLAGS) $(SOURCES) -o $@ $(LFLAGS)
