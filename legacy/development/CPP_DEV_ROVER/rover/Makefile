SOURCES = main.cpp network.cpp camera.cpp
HEADERS = network.h camera.h buffer.h
LIBRARIES = 
CXX_VERSION = -std=c++11
BUILD_FLAGS = $(CXX_VERSION) -Wall -Wextra -Werror -Wpedantic -pedantic-errors -g

all: rover

rover: $(SOURCES) $(HEADERS)
	g++ $(SOURCES) $(BUILD_FLAGS) -o rover

.PHONY: clean
clean:
	rm -f rover
