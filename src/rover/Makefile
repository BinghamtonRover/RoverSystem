SOURCES=camera.cpp main.cpp
HEADERS=camera.hpp
LFLAGS=-L../../bin -lnetwork
CFLAGS=-Wall -Wextra -Wpedantic -std=c++11 -g
EXECUTABLE=rover

$(EXECUTABLE): $(SOURCES) $(HEADERS)
	g++ $(CFLAGS) $(SOURCES) -o $@ $(LFLAGS)

.PHONY: clean
clean:
	rm $(EXECUTABLE)
