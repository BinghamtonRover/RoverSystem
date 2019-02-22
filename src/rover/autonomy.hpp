namespace autonomy {

/*
	Retrieves the most current GPS latitude and longitude coordinates.

	Returns:
		<return>: 	true if the GPS has a fix, false otherwise. If false, 
					the values of out_lat and out_long are undefined and
					should not be used.
		out_lat:	the latitude of the rover if the GPS has a fix.
					This is in degrees, with + for north and - for south.
		out_long:	the longitude of the rover if the GPS has a fix.
					This is in degrees, with + for east and - for west.
*/
bool get_gps_coordinates(float* out_lat, float* out_long);

/*
	Retrieves the most current compass heading of the rover.

	Returns: the heading as degrees clockwise of north.

	Example: 20 degrees north of west is encoded as 290.0f.
*/
float get_heading();

/*
	Retrieves the most current speed of the rover.
	
	Returns: the speed of the rover in meters/second.
*/
float get_speed();

} // namespace autonomy
