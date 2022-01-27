SOURCES = main.cpp camera_feed.cpp gui.cpp debug_console.cpp log_view.cpp waypoint.cpp waypoint_map.cpp shared_feeds.cpp session.cpp controller.cpp controller_manager.cpp
HEADERS = camera_feed.hpp gui.hpp debug_console.hpp log_view.hpp waypoint.hpp waypoint_map.hpp shared_feeds.hpp ../util/util.hpp session.hpp controller.hpp controller_manager.cpp
LIBRARIES = ../../bin/libnetwork.a ../../bin/libsimpleconfig.a ../../bin/liblogger.a
CFLAGS = -std=c++17 -Wall -g -I/opt/libjpeg-turbo/include -pthread
LFLAGS = `pkg-config --static --libs glfw3` -lGL -lX11 -ldl -L/opt/libjpeg-turbo/lib64 -l:libturbojpeg.a -L../../bin -lnetwork -lsimpleconfig -llogger -lcpnetwork
OUT = ../../bin/basestation

$(OUT): $(SOURCES) $(HEADERS) $(LIBRARIES)
	g++ $(CFLAGS) $(SOURCES) -o $@ $(LFLAGS)
