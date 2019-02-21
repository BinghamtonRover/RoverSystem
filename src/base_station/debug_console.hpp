#include "gui.hpp"

#include <string>

namespace gui
{
namespace debug_console
{

void do_debug(gui::Layout *layout, gui::Font *font);

void log(const std::string text, float r, float g, float b);

void handle_input(char c);
void handle_keypress(int key, int mods);

} // namespace debug_console
} // namespace gui
