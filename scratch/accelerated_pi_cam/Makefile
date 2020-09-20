CXX = g++
CXXFLAGS = -DOMX_SKIP64BIT -Wall
LDFLAGS = -L/opt/vc/lib -lopenmaxil -lbcm_host -lvcos

INCLUDES = -I/opt/vc/include

all: Tester

Tester:	omx_encoder.o video_system_exception.o component_event.o omx_video_component.o omx_pi_camera.o video_encoder.o tester.o
	$(CXX) omx_encoder.o video_system_exception.o component_event.o omx_video_component.o omx_pi_camera.o video_encoder.o tester.o $(LDFLAGS) -o $@

tester.o:	tester.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c tester.cpp -o tester.o
	
omx_video_component.o:	omx_video_component.hpp omx_video_component.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c omx_video_component.cpp -o omx_video_component.o

video_encoder.o:	video_encoder.hpp video_encoder.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c video_encoder.cpp -o video_encoder.o

component_event.o:	component_event.hpp component_event.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c component_event.cpp -o component_event.o

omx_pi_camera.o:	omx_pi_camera.hpp omx_pi_camera.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c omx_pi_camera.cpp -o omx_pi_camera.o

video_system_exception.o:	video_system_exception.hpp video_system_exception.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c video_system_exception.cpp -o video_system_exception.o

omx_encoder.o:	omx_encoder.hpp omx_encoder.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c  omx_encoder.cpp -o omx_encoder.o

clean:
	rm -f *.o test
