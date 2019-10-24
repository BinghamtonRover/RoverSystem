#ifndef WAYPOINT_H
#define WAYPOINT_H
#include <vector>

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

void add_waypoint(float lat,float lon);
std::vector<Waypoint> get_waypoints();
void set_rover_coordinates(float lat, float lon);

} //namespace waypoint
#endif
