#include "gui.hpp"

#include <string>
#include <vector>

namespace gui {
namespace debug_console {

typedef void (*CommandCallback)(std::string command);

void set_callback(CommandCallback callback);

void do_debug(gui::Layout* layout, gui::Font* font);

void log(const std::string text, float r, float g, float b);

void handle_input(char c);
void handle_keypress(int key, int mods);

std::vector<std::string> split_by_spaces(std::string s);

} // namespace debug_console
} // namespace gui
