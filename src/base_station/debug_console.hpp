#ifndef DEBUG_CONSOLE_H
#define DEBUG_CONSOLE_H

#include "gui.hpp"
#include "../network/network.hpp"

#include <string>
#include <vector>

namespace gui {
namespace debug_console {

typedef void (*CommandCallback)(std::string command);

void set_callback(CommandCallback callback);

void do_debug(gui::Layout* layout, Font* font);

void log(const std::string text, float r, float g, float b);

void handle_input(char c);
void handle_keypress(int key, int mods);

std::vector<std::string> split_by_spaces(std::string s);

void move(std::vector<std::string> parts, network::MovementMessage last_movement_message);

void mode(std::vector<std::string> parts, network::Feed bs_feed);

} // namespace debug_console
} // namespace gui

#endif