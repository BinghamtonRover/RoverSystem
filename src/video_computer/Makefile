SOURCES = main.cpp camera.cpp session.cpp accelerated_pi_cam/*.cpp
HEADERS = camera.hpp session.hpp ../util/util.hpp accelerated_pi_cam/*.hpp
LIBRARIES = ../../bin/libnetwork.a ../../bin/libsimpleconfig.a ../../bin/liblogger.a
LFLAGS = -L../../bin -lnetwork -L/opt/libjpeg-turbo/lib64 -l:libturbojpeg.a -lsimpleconfig -llogger -L/opt/vc/lib -lopenmaxil -lbcm_host -lvcos
CFLAGS = -pthread -Wall -std=c++11 -g -I/opt/libjpeg-turbo/include -I/opt/vc/include -DOMX_SKIP64BIT
EXECUTABLE = ../../bin/video_computer

$(EXECUTABLE): $(SOURCES) $(HEADERS) $(LIBRARIES)
	g++ $(CFLAGS) $(SOURCES) -o $@ $(LFLAGS)

.PHONY: clean
clean:
	rm $(EXECUTABLE)