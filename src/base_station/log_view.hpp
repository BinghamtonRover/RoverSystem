#ifndef LOG_VIEW_HPP
#define LOG_VIEW_HPP

#include "gui.hpp"
#include "../logger/logger.hpp"


#include <string>

namespace gui {
namespace log_view {

void print(Font* font, int width, std::string m, float r, float g, float b, float a);

void moveUpOne();
void moveDownOne();
void moveTop();
void moveBottom();
void log_view_handler(logger::Level level, std::string message);

void do_log(gui::Layout* layout, int width, int height, Font* font);

} // namespace log_view
} // namespace gui
#endif