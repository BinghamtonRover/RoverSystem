SOURCES = messages.cpp network.cpp stream.cpp
HEADERS = messages.hpp network_util.hpp network.hpp stream.hpp
CFLAGS  = -std=c++17 -Wall -O3 -I/usr/include/boost -I.
OUTDIR  = ../../bin
AFILE   = $(OUTDIR)/libcpnetwork.a
OFILES  = $(SOURCES:%.cpp=$(OUTDIR)/%.o)

$(AFILE): $(OFILES)
	ar rcs $@ $^
	rm $^

$(OUTDIR)/%.o: %.cpp $(HEADERS)
	g++ $(CFLAGS) -c $< -o $@

.PHONY: clean
clean:
	rm $(AFILE)
