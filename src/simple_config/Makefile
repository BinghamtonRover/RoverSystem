SOURCES = simpleconfig.cpp
HEADERS = simpleconfig.h
CFLAGS = -std=c++11 -Wall -g
LFLAGS = 
OBJ_OUT = ../../bin/simpleconfig.o
OUT = ../../bin/libsimpleconfig.a

all: $(OUT)

$(OUT): $(SOURCES) $(HEADERS)
	g++ -c -o $(OBJ_OUT) $(CFLAGS) $(SOURCES) $(LFLAGS)
	ar rcs $(OUT) $(OBJ_OUT)
	rm $(OBJ_OUT)

.PHONY: test
test: $(OUT)
	gcc -o test test.cpp -L./ -l:$(OUT)
	./test
