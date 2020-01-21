SOURCES = main.cpp
HEADERS = ../../src/rocs/rocs.hpp
LFLAGS = -L../../bin -lrocs
CFLAGS = -std=c++11 -Wall -g
OUT = i2ctest

$(OUT): $(SOURCES) $(HEADERS)
	g++ -o $@ $(CFLAGS)  $(SOURCES) $(LFLAGS)
