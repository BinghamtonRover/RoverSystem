#ifndef WAYPOINT_H
#define WAYPOINT_H
#include <vector>

#include "../network/network.hpp"

namespace waypoint
{
struct Waypoint
{
	float latitude;
	float longitude;
};

extern std::vector<Waypoint> cur_waypoints;
extern float rover_latitude;
extern float rover_longitude;
extern network::LocationMessage::FixStatus rover_fix;

void add_waypoint(float lat,float lon);
std::vector<Waypoint> get_waypoints();
void set_rover_coordinates(float lat, float lon);

} //namespace waypoint
#endif
