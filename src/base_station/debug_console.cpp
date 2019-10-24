#include "debug_console.hpp"
#include "gui.hpp"
#include "log_view.hpp"
#include "waypoint.hpp"
#include "logger.hpp"

#include <GLFW/glfw3.h>

#include <string>
#include <vector>

namespace gui {
namespace debug_console {

const int CONSOLE_WIDTH = WINDOW_WIDTH;
const int CONSOLE_HEIGHT = WINDOW_HEIGHT / 5;

const float CONSOLE_BACKGROUND_SHADE = 40.0f / 255.0f;

const int CONSOLE_TEXT_SIZE = 16;

const int CONSOLE_MARGIN = 5;
const int CONSOLE_PADDING = 2;

const int CONSOLE_CURSOR_PADDING = 2;

const std::string CONSOLE_PROMPT = "> ";

struct DebugLine {
    std::string line;

    float r = 0.0f, g = 1.0f, b = 0.0f;
};

struct DebugConsole {
    std::string prompt;
    Font font;

    DebugLine buffer = { CONSOLE_PROMPT, 0, 1, 0 };
    std::vector<DebugLine> history;

    CommandCallback callback;
} console;

void set_callback(CommandCallback callback) {
    console.callback = callback;
}

void do_debug(gui::Layout* layout, gui::Font* font) {
    if (!gui::state.show_debug_console) {
        return;
    }

    if (glfwGetKey(gui::state.window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        gui::state.show_debug_console = false;
        gui::state.input_state = InputState::KEY_COMMAND;
        return;
    }

    gui::do_solid_rect(
        layout,
        CONSOLE_WIDTH,
        CONSOLE_HEIGHT,
        CONSOLE_BACKGROUND_SHADE,
        CONSOLE_BACKGROUND_SHADE,
        CONSOLE_BACKGROUND_SHADE);

    // Begin the text half a character above the bottom of debug.
    int text_begin = CONSOLE_HEIGHT - 1.5 * CONSOLE_TEXT_SIZE;

    // Draw the buffer line first then add a gap between this line and the next.
    glColor4f(console.buffer.r, console.buffer.g, console.buffer.b, 1.0f);
    gui::draw_text(font, console.buffer.line.c_str(), CONSOLE_MARGIN, text_begin, CONSOLE_TEXT_SIZE);

    int btw = CONSOLE_MARGIN + gui::text_width(font, console.buffer.line.c_str(), CONSOLE_TEXT_SIZE);
    // Size the cursor according to the size of an 'm'.
    int cw = gui::text_width(font, "m", CONSOLE_TEXT_SIZE);

    // Now there's this thing...
    // If the last character of the string is a space, then text_width is wrong.
    // Its not our fault.
    if (console.buffer.line[console.buffer.line.size() - 1] == ' ') {
        btw += cw;
    }

    // Add padding.
    btw += CONSOLE_CURSOR_PADDING;

    // Draw the cursor.
    glBegin(GL_QUADS);

    glVertex2f(btw, text_begin);
    glVertex2f(btw + cw, text_begin);
    glVertex2f(btw + cw, text_begin + CONSOLE_TEXT_SIZE);
    glVertex2f(btw, text_begin + CONSOLE_TEXT_SIZE);

    glEnd();

    text_begin -= CONSOLE_TEXT_SIZE + CONSOLE_PADDING;

    // Write the history to the console until out of history or top of the screen is reached.
    for (int i = console.history.size() - 1; i >= 0; i--) {
        DebugLine* line = &console.history[i];

        glColor4f(line->r, line->g, line->b, 1.0f);
        gui::draw_text(font, line->line.c_str(), CONSOLE_MARGIN, text_begin, CONSOLE_TEXT_SIZE);

        // Move up and add a gap between lines.
        text_begin -= CONSOLE_TEXT_SIZE + CONSOLE_PADDING;

        if (text_begin < 0) break;
    }
}

void log(std::string text, float r, float g, float b) {
    if (text.size() < 200) {
        DebugLine line{ text, r, g, b };
        console.history.push_back(line);
        return;
    }

    while (text.size() >= 200) {
        for (int i = 200; i > 0; i--) {
            if (text[i] == ' ') {
                std::string l = text.substr(0, i);
                DebugLine line{ l, r, g, b };
                console.history.push_back(line);
                text = text.substr(i, text.size());
                break;
            }
        }
    }
}

void handle_input(char c) {
    console.buffer.line.push_back(c);
}

void handle_keypress(int key, int mods) {
    if (key == GLFW_KEY_ENTER) {
        std::string command = console.buffer.line.substr(CONSOLE_PROMPT.size());

        console.history.push_back(console.buffer);
        console.buffer = { CONSOLE_PROMPT, 0, 1, 0 };

        console.callback(command);

        if (command == "test") {
            log("This is some red text.", 1, 0, 0);
        } else if (command == "tdl") {
            bool mode = logger::toggleDebugMode();
            if(mode) {
                log("DebugMode turned on", 1, 0, 1);
            } else {
                log("DebugMode turned off", 1, 0, 1);
            }
        } else if (command.substr(0, 3) == "aw ") {
            std::vector<double> wps;
            int space = 0;
            space = command.substr(3).find(" ");
            if(space <= 3) {
                log("Invalid Input: \"aw\" takes two doubles, for example: \"aw 42.2 75.3\"", 1, 0, 0);
            } else {
                float lat = atof(command.substr(3, space).c_str());
                float lon = atof(command.substr(space+4).c_str());
                waypoint::add_waypoint(lat,lon);
                log("Waypoint [" + std::to_string(lat) + ", " + std::to_string(lon) + "] added.", 1, 0, 1);
            }
        } else if (command == "aw") {
            log("Invalid Input: \"aw\" takes two doubles, for example: \"aw 42.2 75.3\"", 1, 0, 0);
        } else if (command == "lw") {
            log("Waypoints: ", 1, 0, 1);
            auto waypoints = waypoint::get_waypoints();
            for(unsigned int i = 0; i < waypoints.size(); i++) {
                std::string latStr = std::to_string(waypoints.at(i).latitude);
                std::string lonStr = std::to_string(waypoints.at(i).longitude);
                log("[" + latStr + ", " + lonStr + "]", 1, 1, 1);
            }
        } else if (command == "help") {
            log("------------------", 1, 1, 1);
            log("Command List: ", 1, 1, 1);
            log("------------------", 1, 1, 1);
            log("aw = Add Waypoint", 1, 1, 1);
            log("help = List Debug Menu Commands", 1, 1, 1);
            log("lw = List Waypoints", 1, 1, 1);
            log("tdl = Toggle Debug Log", 1, 1, 1);
            log("test = Test Debug Menu", 1, 1, 1);
        } else if (command == "") {
            log("No command entered.", 1, 0, 0);
        } else {
            log("Invalid command.", 1, 0, 0);
        }
    } else if (key == GLFW_KEY_BACKSPACE) {
        if (mods & GLFW_MOD_CONTROL) {
            console.buffer = { CONSOLE_PROMPT, 0, 1, 0 };
        } else {
            int real_len = console.buffer.line.size() - CONSOLE_PROMPT.size();
            if (real_len > 0) {
                console.buffer.line.pop_back();
            }
        }
    } else if (key == GLFW_KEY_ESCAPE) {
        gui::state.input_state = gui::InputState::KEY_COMMAND;
        gui::state.show_debug_console = false;
    }
}

} // namespace debug_console
} // namespace gui
