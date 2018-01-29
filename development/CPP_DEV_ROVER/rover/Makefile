SOURCES = main.cpp network.cpp
HEADERS = network.h
CXX_VERSION = -std=c++14
BUILD_FLAGS = $(CXX_VERSION) -Wall -Wextra -Werror -Wpedantic -pedantic-errors -g

all: rover

rover: $(SOURCES) $(HEADERS)
	g++ $(SOURCES) $(BUILD_FLAGS) -o rover

.PHONY: clean

clean:
	rm -f rover