SOURCES = main.cpp camera.cpp serial.cpp
HEADERS = camera.hpp autonomy.hpp serial.hpp suspension.hpp
LIBRARIES = ../../bin/libnetwork.a
LFLAGS = -L../../bin -lnetwork -L/opt/libjpeg-turbo/lib64 -l:libturbojpeg.a
CFLAGS = -Wall -Wextra -Wpedantic -std=c++11 -g -I/opt/libjpeg-turbo/include
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
ifeq($(SUSPENSION_BACKEND), old)
	SOURCES += suspension_old.cpp
endif

$(EXECUTABLE): $(SOURCES) $(HEADERS) $(LIBRARIES)
	g++ $(CFLAGS) $(SOURCES) -o $@ $(LFLAGS)

run: $(EXECUTABLE)
	./$(EXECUTABLE)

.PHONY: clean
clean:
	rm $(EXECUTABLE)
