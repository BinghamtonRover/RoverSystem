SOURCES = network.cpp memory.cpp
HEADERS = network.hpp memory.hpp
CFLAGS  = -std=c++14 -Wall -g
OUTDIR  = ../../bin
AFILE   = $(OUTDIR)/libnetwork.a
OFILES  = $(SOURCES:%.cpp=$(OUTDIR)/%.o)

$(AFILE): $(OFILES)
	ar rcs $@ $^
	rm $^

$(OUTDIR)/%.o: %.cpp $(HEADERS)
	g++ $(CFLAGS) -c $< -o $@

.PHONY: clean
clean:
	rm $(AFILE)
