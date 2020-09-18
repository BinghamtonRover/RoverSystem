CXX = g++
CXXFLAGS = -DOMX_SKIP64BIT -Wall
LDFLAGS = -L/opt/vc/lib -lopenmaxil -lbcm_host -lvcos

INCLUDES = -I/opt/vc/include

all: Tester

Tester:	component_event.o omx_video_component.o video_encoder.o tester.o
	$(CXX) component_event.o omx_video_component.o video_encoder.o tester.o $(LDFLAGS) -o $@

tester.o:	tester.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c tester.cpp -o tester.o
	
omx_video_component.o:	omx_video_component.hpp omx_video_component.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c omx_video_component.cpp -o omx_video_component.o

video_encoder.o:	video_encoder.hpp video_encoder.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c video_encoder.cpp -o video_encoder.o

component_event.o:	component_event.hpp component_event.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c component_event.cpp -o component_event.o


clean:
	rm -f *.o test
