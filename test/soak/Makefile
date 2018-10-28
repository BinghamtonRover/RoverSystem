SOURCES = connectionTest.cpp
CFLAGS = -std=c++14 -Wall -g
LFLAGS = -L../../bin/ -lnetwork
OUT = ../../bin/connectiontest

$(OUT): $(SOURCES)
	g++ -o $@ $(CFLAGS) $(SOURCES) $(LFLAGS)
