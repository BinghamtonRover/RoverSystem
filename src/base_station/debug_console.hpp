#ifndef DEBUG_CONSOLE_H
#define DEBUG_CONSOLE_H

#include "session.hpp"
#include "gui.hpp"
#include "../network/network.hpp"

#include <string>
#include <vector>

namespace gui {
namespace debug_console {

typedef void (*CommandCallback)(std::string command, Session *session);

void set_callback(CommandCallback callback);

void command_callback(std::string command, Session *bs_session);

void do_debug(gui::Layout* layout, Font* font);

void log(const std::string text, float r, float g, float b);

void handle_input(char c);
void handle_keypress(int key, int mods, Session *session);

std::vector<std::string> split_by_spaces(std::string s);

} // namespace debug_console
} // namespace gui

#endif
