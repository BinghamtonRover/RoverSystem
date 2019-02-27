SOURCES = main.cpp camera.cpp lidar.cpp
HEADERS = camera.hpp autonomy.hpp suspension.hpp lidar.hpp
LIBRARIES = ../../bin/libnetwork.a
LFLAGS = -L../../bin -lnetwork -L/opt/libjpeg-turbo/lib64 -l:libturbojpeg.a
LFLAGS += -L/usr/local/zed/lib -rdynamic /usr/local/zed/lib/libsl_input.so /usr/local/zed/lib/libsl_core.so /usr/local/zed/lib/libsl_zed.so -lcuda /usr/local/cuda/lib64/libcudart.so /usr/local/cuda-9.0/lib64/libnppial.so /usr/local/cuda-9.0/lib64/libnppisu.so /usr/local/cuda-9.0/lib64/libnppicc.so /usr/local/cuda-9.0/lib64/libnppicom.so /usr/local/cuda-9.0/lib64/libnppidei.so /usr/local/cuda-9.0/lib64/libnppif.so /usr/local/cuda-9.0/lib64/libnppig.so /usr/local/cuda-9.0/lib64/libnppim.so /usr/local/cuda-9.0/lib64/libnppist.so /usr/local/cuda-9.0/lib64/libnppitc.so /usr/local/cuda/lib64/libnpps.so -Wl,-rpath,/usr/local/zed/lib:/usr/local/cuda/lib64:/usr/local/cuda-9.0/lib64 
CFLAGS = -Wall -Wextra -Wpedantic -std=c++11 -g -I/opt/libjpeg-turbo/include
CFLAGS += -Wno-reorder -I/usr/local/zed/include -I/usr/local/cuda-9.0/include
EXECUTABLE = ../../bin/rover

AUTONOMY_BACKEND = dummy
SUSPENSION_BACKEND = dummy

ifeq ($(AUTONOMY_BACKEND), dummy)
	SOURCES += autonomy_dummy.cpp
# else ifeq ($(AUTONOMY_BACKEND), rover)
#	LIBRARIES += <path_to_autonomy_library>
#	LFLAGS += -L<path_to_autonomy_library> -l:libautonomy.a
endif

ifeq ($(SUSPENSION_BACKEND), dummy)
	SOURCES += suspension_dummy.cpp
else 
ifeq ($(SUSPENSION_BACKEND), old)
	SOURCES += suspension_old.cpp
endif
endif

$(EXECUTABLE): $(SOURCES) $(HEADERS) $(LIBRARIES)
	g++ $(CFLAGS) $(SOURCES) -o $@ $(LFLAGS)

run: $(EXECUTABLE)
	./$(EXECUTABLE)

.PHONY: clean
clean:
	rm $(EXECUTABLE)
