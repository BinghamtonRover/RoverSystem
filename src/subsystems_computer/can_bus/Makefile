SOURCES = can_controller.cpp
HEADERS = can_controller.hpp
CFLAGS = -std=c++11 -Wall -g
LFLAGS = 

all:	can_controller.o
	mv can_controller.o ../../../bin
#I know this looks cursed, but I'm doing this to store the ".o" files with the ".a" files
can_controller.o:	can_controller.cpp
	g++ -c can_controller.cpp



clean:
	@rm -f *.o