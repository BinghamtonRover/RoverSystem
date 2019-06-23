ROOT = ../..
SOURCES = main.cpp camera.cpp 
HEADERS = camera.hpp autonomy.hpp suspension.hpp lidar.hpp gps.hpp
LIBRARIES = $(ROOT)/bin/libnetwork.a
LFLAGS = -L$(ROOT)/bin -lnetwork -L/opt/libjpeg-turbo/lib64 -l:libturbojpeg.a
CFLAGS = -no-pie -pthread -Wall -std=c++11 -g -I/opt/libjpeg-turbo/include -I$(ROOT)/vendor
EXECUTABLE = $(ROOT)/bin/rover

P = d # Platform. d is dummy, r is rover.

ifeq ($(P), r)
	# Vendored libs.
	LFLAGS += -L$(ROOT)/vendor/arm

	# Simpleconfig.
	LFLAGS += -l:libsimpleconfig.a

	# Sensors
	SOURCES += lidar_onboard.cpp
	SOURCES += zed_onboard.cpp
	SOURCES += gps_onboard.cpp

	# Zed dependencies
	LFLAGS += -L/usr/local/zed/lib -rdynamic /usr/local/zed/lib/libsl_input.so /usr/local/zed/lib/libsl_core.so /usr/local/zed/lib/libsl_zed.so -lcuda /usr/local/cuda/lib64/libcudart.so /usr/local/cuda-9.0/lib64/libnppial.so /usr/local/cuda-9.0/lib64/libnppisu.so /usr/local/cuda-9.0/lib64/libnppicc.so /usr/local/cuda-9.0/lib64/libnppicom.so /usr/local/cuda-9.0/lib64/libnppidei.so /usr/local/cuda-9.0/lib64/libnppif.so /usr/local/cuda-9.0/lib64/libnppig.so /usr/local/cuda-9.0/lib64/libnppim.so /usr/local/cuda-9.0/lib64/libnppist.so /usr/local/cuda-9.0/lib64/libnppitc.so /usr/local/cuda/lib64/libnpps.so -Wl,-rpath,/usr/local/zed/lib:/usr/local/cuda/lib64:/usr/local/cuda-9.0/lib64 
	CFLAGS += -Wno-reorder -I/usr/local/zed/include -I/usr/local/cuda-9.0/include

	# Suspension
	SOURCES += suspension_old.cpp

	# Autonomy
	SOURCES += autonomy_onboard.cpp
else
	# Vendored libs.
	LFLAGS += -L$(ROOT)/vendor/x86_64

	# Simpleconfig.
	LFLAGS += -l:libsimpleconfig.a

	# Sensors
	SOURCES += lidar_dummy.cpp
	SOURCES += zed_dummy.cpp
	SOURCES += gps_dummy.cpp

	# Suspension
	SOURCES += suspension_dummy.cpp

	# Autonomy
	SOURCES += autonomy_dummy.cpp
endif

$(EXECUTABLE): $(SOURCES) $(HEADERS) $(LIBRARIES)
	g++ $(CFLAGS) $(SOURCES) -o $@ $(LFLAGS)

run: $(EXECUTABLE)
	./$(EXECUTABLE)

.PHONY: clean
clean:
	rm $(EXECUTABLE)
