SOURCES = main.cpp camera.cpp subsystem.cpp
HEADERS = camera.hpp gps.hpp subsystem.hpp ../util/util.hpp lidar.hpp
LIBRARIES = ../../bin/libnetwork.a ../../bin/libsimpleconfig.a ../../bin/liblogger.a
LFLAGS = -L../../bin -lnetwork -L/opt/libjpeg-turbo/lib64 -l:libturbojpeg.a -lsimpleconfig -llogger
CFLAGS = -pthread -Wall -std=c++11 -g -I/opt/libjpeg-turbo/include
EXECUTABLE = ../../bin/rover

ifdef ONBOARD
	SUSPENSION_ONBOARD = true
	ROCS_ONBOARD = true
	LIDAR_ONBOARD = true
	GPS_ONBOARD = true
endif

ifdef SUSPENSION_ONBOARD
	# ROCS is a dependency
	ROCS_ONBOARD = true
	SOURCES += suspension_onboard.cpp
else
	SOURCES += suspension_dummy.cpp
endif

ifdef ROCS_ONBOARD
	LIBRARIES += ../../bin/librocs.a
	LFLAGS += -lrocs
else
	SOURCES += rocs_dummy.cpp
endif

ifdef LIDAR_ONBOARD
	SOURCES += lidar_onboard.cpp
else
	SOURCES += lidar_dummy.cpp
endif

ifdef GPS_ONBOARD
	SOURCES += gps_onboard.cpp
else
	SOURCES += gps_dummy.cpp
endif

$(EXECUTABLE): $(SOURCES) $(HEADERS) $(LIBRARIES)
	g++ $(CFLAGS) $(SOURCES) -o $@ $(LFLAGS)

.PHONY: clean
clean:
	rm $(EXECUTABLE)
