LFLAGS = -L../../bin/ -lnetwork
MAIN = controller

all: $(MAIN)

$(MAIN): $(MAIN).o
	g++ -Wall $(MAIN).o $(LFLAGS) -o $(MAIN)

$(MAIN).o: $(MAIN).cpp
	g++ -c $(LFLAGS) $(MAIN).cpp

clean:
	rm $(MAIN).o $(MAIN)
