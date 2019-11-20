#include "waypoint.hpp"
#include "gui.hpp"

namespace waypoint_menu {

	extern bool select;
	extern bool add_waypoint;
	extern int selection;
	extern int current_waypoints;
	extern int max_waypoints;
	extern bool lat_box;
	extern unsigned int max_characters;
	extern std::string lat_buffer;
	extern std::string lon_buffer;
	void do_menu(gui::Font * font, float x, float y);
	void add_character(int key); //Keys: "0-9" = 48-57, "Period" = 46, "Minus" = 45

}//namespace waypoint_menu