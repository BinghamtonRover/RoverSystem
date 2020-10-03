SOURCES = main.cpp camera.cpp session.cpp
HEADERS = camera.hpp session.hpp ../util/util.hpp
LIBRARIES = ../../bin/libnetwork.a ../../bin/libsimpleconfig.a ../../bin/liblogger.a
LFLAGS = -L../../bin -lnetwork -L/opt/libjpeg-turbo/lib64 -l:libturbojpeg.a -lsimpleconfig -llogger
CFLAGS = -pthread -Wall -std=c++11 -g -I/opt/libjpeg-turbo/include
EXECUTABLE = ../../bin/video_computer

$(EXECUTABLE): $(SOURCES) $(HEADERS) $(LIBRARIES)
	g++ $(CFLAGS) $(SOURCES) -o $@ $(LFLAGS)

.PHONY: clean
clean:
	rm $(EXECUTABLE)