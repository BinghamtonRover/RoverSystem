SOURCES = main.cpp subsystem.cpp session.cpp
HEADERS = gps.hpp subsystem.hpp ../util/util.hpp lidar.hpp session.hpp
LIBRARIES = ../../bin/libnetwork.a ../../bin/libsimpleconfig.a ../../bin/liblogger.a
LFLAGS = -L../../bin -lnetwork -lsimpleconfig -llogger
CFLAGS = -pthread -Wall -std=c++11 -g 
EXECUTABLE = ../../bin/subsystem_computer

ifdef ONBOARD
	SUSPENSION_ONBOARD = true
	ROCS_ONBOARD = true
	LIDAR_ONBOARD = true
	GPS_ONBOARD = true
	ARM_ONBOARD = true
#	GRIPPER_ONBOARD = true
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

ifdef ARM_ONBOARD
	SOURCES += arm_onboard.cpp
else
	SOURCES += arm_dummy.cpp
endif

#ifdef GRIPPER_ONBOARD
#	SOURCES += gripper_onboard.cpp
#else
#	SOURCES += gripper_dummy.cpp
#endif

#always add CAN to onboard, until a dummy (if necessary) needs to be created
#SOURCES += ../../bin/can_controller.o

$(EXECUTABLE): $(SOURCES) $(HEADERS) $(LIBRARIES)
	g++ $(CFLAGS) $(SOURCES) -o $@ $(LFLAGS)

.PHONY: clean
clean:
	rm $(EXECUTABLE)
