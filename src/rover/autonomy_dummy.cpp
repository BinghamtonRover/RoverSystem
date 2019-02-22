#include "autonomy.hpp"

/*
	This file contains dummy values for now.
*/

namespace autonomy {

bool get_gps_coordinates(float* out_lat, float* out_long) {
	// This is the GPS location of the IEEE lab.
	*out_lat = 42.087046f;
	*out_long = -75.967886f;

	return true;
}

float get_heading() {
	return 6.2f;
}

float get_speed() {
	return 5.3f;
}

} // namespace autonomy
