SOURCES = main.cpp
CFLAGS = -std=c++11 -Wall -g -I/usr/local/include/opencv4
LFLAGS = -L/usr/local/lib/ -lopencv_core -lopencv_highgui -lopencv_aruco -lopencv_videoio -lopencv_imgproc
OUTDIR = ../../bin
OUT = $(OUTDIR)/markers

$(OUT): $(SOURCES)
	g++ -o $@ $(CFLAGS) $(SOURCES) $(LFLAGS)
