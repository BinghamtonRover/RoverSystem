SOURCES = network.cpp
HEADERS = network.hpp arena.hpp
CFLAGS  = -std=c++11 -Wall
OUTDIR  = ../../bin
OFILE   = $(OUTDIR)/network.o
AFILE   = $(OUTDIR)/libnetwork.a

$(AFILE): $(OFILE)
	ar rcs $@ $^
	rm $(OFILE)

$(OFILE): $(SOURCES) $(HEADERS) Makefile
	g++ $(CFLAGS) -c $(SOURCES) -o $@

.PHONY: clean
clean:
	rm $(AFILE)