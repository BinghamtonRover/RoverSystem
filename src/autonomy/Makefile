SOURCES = autonomy.cpp 
HEADERS = autonomy.hpp
CFLAGS  = -std=c++14 -Wall -g
OUTDIR  = ../../bin
AFILE   = $(OUTDIR)/libautonomy.a
OFILES  = $(SOURCES:%.cpp=$(OUTDIR)/%.o)

$(AFILE): $(OFILES)
	ar rcs $@ $^
	rm $^

$(OUTDIR)/%.o: %.cpp $(HEADERS)
	g++ $(CFLAGS) -c $< -o $@

.PHONY: clean
clean:
	rm $(AFILE) 
