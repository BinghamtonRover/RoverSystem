SOURCES = main.cpp
HEADERS = 
LIBRARIES = ../../bin/libnetwork.a
CFLAGS = -std=c++14 -Wall -g
LFLAGS = -L../../bin -lnetwork 
OUT = ../../bin/new_network_test

$(OUT): $(SOURCES) $(HEADERS) $(LIBRARIES)
	g++ $(CFLAGS) $(SOURCES) -o $@ $(LFLAGS)
