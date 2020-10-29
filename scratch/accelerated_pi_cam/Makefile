CXX = g++
CXXFLAGS = -DOMX_SKIP64BIT -Wall
LDFLAGS = -L/opt/vc/lib -lopenmaxil -lbcm_host -lvcos

INCLUDES = -I/opt/vc/include -I../../src/network
VID_OBJECTS = \
	omx_video_component.o \
	video_system.o \
	component_event.o \
	omx_pi_camera.o \
	video_system_exception.o \
	omx_encoder.o

all: tester netsample

tester:	$(VID_OBJECTS) tester.o
	$(CXX) $(LDFLAGS) $(VID_OBJECTS) tester.o -o $@

netsample: $(VID_OBJECTS) network_sample.o
	$(CXX) $(LDFLAGS) $(VID_OBJECTS) network_sample.o -o network_sample

tester.o: tester.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c tester.cpp -o tester.o

network_sample.o: network_sample.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c network_sample.cpp
	
omx_video_component.o: omx_video_component.hpp omx_video_component.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c omx_video_component.cpp

video_system.o:	video_system.hpp video_system.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c video_system.cpp

component_event.o: component_event.hpp component_event.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c component_event.cpp

omx_pi_camera.o: omx_pi_camera.hpp omx_pi_camera.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c omx_pi_camera.cpp

video_system_exception.o: video_system_exception.hpp video_system_exception.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c video_system_exception.cpp

omx_encoder.o: omx_encoder.hpp omx_encoder.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c omx_encoder.cpp

clean:
	rm -rf *.o tester network_sample video.h264
