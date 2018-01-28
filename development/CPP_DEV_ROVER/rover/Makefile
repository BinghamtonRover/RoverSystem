SOURCES = main.cpp network.cpp
HEADERS = network.h

all: rover

rover: $(SOURCES) $(HEADERS)
	g++ -Wall -o rover $(SOURCES)

.PHONY: clean
clean:
	rm rover