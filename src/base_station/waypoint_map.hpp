#include "gui.hpp"

namespace gui {
namespace waypoint_map {

//determines the zoom for the waypoint map
const float PPM_MIN = 0.09f;
const float PPM_MAX = 2.02f;
const float PPM_SCALE_FACTOR = 1.03f;

void do_waypoint_map(gui::Layout* layout, int w, int h);

void zoom_in();
void zoom_out();

}}
