LFLAGS = -L../../bin/ -lnetwork

all: connectionTest

connectionTest: connectionTest.o
	g++ -Wall connectionTest.o $(LFLAGS) -o connectionTest

connectionTest.o: connectionTest.cpp
	g++ -c $(LFLAGS) connectionTest.cpp

clean:
	rm connectionTest.o connectionTest
