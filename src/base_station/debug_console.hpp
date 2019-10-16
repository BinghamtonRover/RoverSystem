#include "gui.hpp"

#include <string>
#include <vector>

namespace gui
{
namespace debug_console
{

struct Waypoint
{
	float latitude;
	float longitude;
};

typedef void (*CommandCallback)(std::string command);

void set_callback(CommandCallback callback);

void do_debug(gui::Layout *layout, gui::Font *font);

void log(const std::string text, float r, float g, float b);

void handle_input(char c);
std::vector<Waypoint> handle_keypress(int key, int mods);

} // namespace debug_console
} // namespace gui
