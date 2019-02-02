SOURCES = main.cpp camera.cpp
HEADERS = camera.hpp
LIBRARIES = ../../bin/libnetwork.a
LFLAGS = -L../../bin -lnetwork -L/opt/libjpeg-turbo/lib64 -l:libturbojpeg.a
CFLAGS = -Wall -Wextra -Wpedantic -std=c++11 -g -I/opt/libjpeg-turbo/include
EXECUTABLE = ../../bin/rover

$(EXECUTABLE): $(SOURCES) $(HEADERS) $(LIBRARIES)
	g++ $(CFLAGS) $(SOURCES) -o $@ $(LFLAGS)

run: $(EXECUTABLE)
	./$(EXECUTABLE)

.PHONY: clean
clean:
	rm $(EXECUTABLE)