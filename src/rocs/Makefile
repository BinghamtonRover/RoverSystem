SOURCES = rocs.cpp
HEADERS = rocs.hpp
CFLAGS = -std=c++11 -Wall -g
LFLAGS = 
OBJ_OUT = ../../bin/librocs.o
OUT = ../../bin/librocs.a

all: $(OUT)

$(OUT): $(SOURCES) $(HEADERS)
	g++ -c -o $(OBJ_OUT) $(CFLAGS) $(SOURCES) $(LFLAGS)
	ar rcs $(OUT) $(OBJ_OUT)
