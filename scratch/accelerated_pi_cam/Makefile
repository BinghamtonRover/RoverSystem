CXX = g++
CXXFLAGS = -DOMX_SKIP64BIT

INCLUDES = -I/opt/vc/include

all: VideoEncoder

VideoEncoder:	omx_video_component.o video_encoder.o
	
omx_video_component.o:	omx_video_component.hpp omx_video_component.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c omx_video_component.cpp -o omx_video_component.o

video_encoder.o:	video_encoder.hpp video_encoder.cpp omx_video_component.o
	$(CXX) $(CXXFLAGS) $(INCLUDES) omx_video_component.o -c video_encoder.cpp -o video_encoder.o


clean:
	rm -f *.o test
