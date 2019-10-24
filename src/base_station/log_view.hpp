#include "gui.hpp"

#include <string>

namespace gui {
namespace log_view {

void print(gui::Font* font, int width, std::string m, float r, float g, float b, float a);

void moveUpOne();
void moveDownOne();
void moveTop();
void moveBottom();

void do_log(gui::Layout* layout, int width, int height, gui::Font* font);

} // namespace log_view
} // namespace gui
