#include "gui.hpp"

#include <string>

namespace gui {
namespace log_view {

void print(std::string m, float r, float g, float b, float a);

void calc_sizing(gui::Font* font, int width, int height);

void moveUpOne();
void moveDownOne();
void moveTop();
void moveBottom();

void do_log(gui::Layout* layout, int width, int height, gui::Font* font);

} } // namespace gui::log_view
