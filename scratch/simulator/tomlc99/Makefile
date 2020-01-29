SOURCES = toml.c 
HEADERS = toml.h
CFLAGS  = -std=c99 -Wall -g
OUTDIR  = ../../../bin
AFILE   = $(OUTDIR)/libtoml.a
OFILES  = $(SOURCES:%.c=$(OUTDIR)/%.o)

$(AFILE): $(OFILES)
	ar rcs $@ $^
	rm $^

$(OUTDIR)/%.o: %.c $(HEADERS)
	gcc $(CFLAGS) -c $< -o $@

.PHONY: clean
clean:
	rm $(AFILE) 
