SOURCES = main.cpp camera.cpp subsystem.cpp
HEADERS = camera.hpp zed.hpp gps.hpp subsystem.hpp ../util/util.hpp 
LIBRARIES = ../../bin/libnetwork.a ../../bin/libsimpleconfig.a
LFLAGS = -L../../bin -lnetwork -L/opt/libjpeg-turbo/lib64 -l:libturbojpeg.a -lsimpleconfig
CFLAGS = -pthread -Wall -std=c++11 -g -I/opt/libjpeg-turbo/include
EXECUTABLE = ../../bin/rover

ifdef $(ONBOARD)
	SOURCES += zed_onboard.cpp
	SOURCES += gps_dummy.cpp

	# Zed dependencies
	LFLAGS += -L/usr/local/zed/lib -rdynamic /usr/local/zed/lib/libsl_input.so /usr/local/zed/lib/libsl_core.so /usr/local/zed/lib/libsl_zed.so -lcuda /usr/local/cuda/lib64/libcudart.so /usr/local/cuda-9.0/lib64/libnppial.so /usr/local/cuda-9.0/lib64/libnppisu.so /usr/local/cuda-9.0/lib64/libnppicc.so /usr/local/cuda-9.0/lib64/libnppicom.so /usr/local/cuda-9.0/lib64/libnppidei.so /usr/local/cuda-9.0/lib64/libnppif.so /usr/local/cuda-9.0/lib64/libnppig.so /usr/local/cuda-9.0/lib64/libnppim.so /usr/local/cuda-9.0/lib64/libnppist.so /usr/local/cuda-9.0/lib64/libnppitc.so /usr/local/cuda/lib64/libnpps.so -Wl,-rpath,/usr/local/zed/lib:/usr/local/cuda/lib64:/usr/local/cuda-9.0/lib64 
	CFLAGS += -Wno-reorder -I/usr/local/zed/include -I/usr/local/cuda-9.0/include
else
	SOURCES += zed_dummy.cpp
	SOURCES += gps_dummy.cpp
endif

$(EXECUTABLE): $(SOURCES) $(HEADERS) $(LIBRARIES)
	g++ $(CFLAGS) $(SOURCES) -o $@ $(LFLAGS)

.PHONY: clean
clean:
	rm $(EXECUTABLE)
