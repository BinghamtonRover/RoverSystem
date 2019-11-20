#include "waypoint.hpp"
#include <vector>


namespace waypoint
{
std::vector<Waypoint> cur_waypoints;
//rover's coordinates initially set to outside the lat/lon range to indicate the coordinates haven't been updated
float rover_latitude = -100;
float rover_longitude = -200;

void add_waypoint(float lat, float lon){
	cur_waypoints.push_back({lat,lon});
}

void remove_waypoint(int position){
	cur_waypoints.erase(cur_waypoints.begin() + position);
}

std::vector<Waypoint> get_waypoints(){
	return cur_waypoints;
}

void set_rover_coordinates(float lat, float lon){
	rover_latitude = lat;
	rover_longitude = lon;
}


} //namespace waypoint