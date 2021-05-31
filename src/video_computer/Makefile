SOURCES = main.cpp camera.cpp session.cpp
HEADERS = camera.hpp session.hpp ../util/util.hpp
LIBRARIES = ../../bin/libnetwork.a ../../bin/libsimpleconfig.a ../../bin/liblogger.a
LFLAGS = -L../../bin -lnetwork -L/opt/libjpeg-turbo/lib64 -l:libturbojpeg.a -lsimpleconfig -llogger
CFLAGS = -pthread -Wall -std=c++11 -g -I/opt/libjpeg-turbo/include
SOURCES_PI = accelerated_pi_cam/*.cpp
HEADERS_PI = accelerated_pi_cam/*.hpp
LFLAGS_PI = -L/opt/vc/lib -lopenmaxil -lbcm_host -lvcos
CFLAGS_PI = -I/opt/vc/include -DOMX_SKIP64BIT -DPI_OMX_DRIVERS
EXECUTABLE = ../../bin/video_computer

$(EXECUTABLE): $(SOURCES) $(HEADERS) $(LIBRARIES)
	g++ $(CFLAGS) $(SOURCES) -o $@ $(LFLAGS)

video_computer_pi: $(SOURCES) $(SOURCES_PI) $(HEADERS) $(HEADERS_PI) $(LIBRARIES)
	g++ $(CFLAGS) $(CFLAGS_PI) $(SOURCES) $(SOURCES_PI) -o $(EXECUTABLE) $(LFLAGS) $(LFLAGS_PI)

.PHONY: clean
clean:
	rm $(EXECUTABLE)