#include "gui.hpp"

namespace gui {
namespace waypoint_map {

//determines the zoom for the waypoint map
const float PPM_MIN = 2.0f;
const float PPM_MAX = 20.0f;
const float PPM_SCALE_FACTOR = 1.03f;

const float GRID_SPACING = 10.0f; // In meters.

void do_waypoint_map(gui::Layout* layout, int w, int h);

void zoom_in();
void zoom_out();

}}
