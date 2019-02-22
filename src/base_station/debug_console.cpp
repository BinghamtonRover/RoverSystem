#include "gui.hpp"
#include "debug_console.hpp"

#include <GLFW/glfw3.h>

#include <string>
#include <vector>

namespace gui
{
namespace debug_console
{

const int CONSOLE_WIDTH = WINDOW_WIDTH;
const int CONSOLE_HEIGHT = WINDOW_HEIGHT / 5;

const int HELP_WIDTH = WINDOW_WIDTH / 8;
const int HELP_HEIGHT = WINDOW_HEIGHT / 3;

const float CONSOLE_BACKGROUND_SHADE = 40.0f / 255.0f;

const int CONSOLE_TEXT_SIZE = 16;

const int CONSOLE_MARGIN = 5;
const int CONSOLE_PADDING = 2;
const int HELP_MARGIN = 100;
const int HELP_PADDING = 2;

const int CONSOLE_CURSOR_PADDING = 2;

const std::string CONSOLE_PROMPT = "> ";

struct DebugLine
{
	std::string line;

	float r = 0.0f, g = 1.0f, b = 0.0f;
};

struct DebugConsole {
    std::string prompt;
    Font font;

    DebugLine buffer = {CONSOLE_PROMPT, 0, 1, 0};
    std::vector<DebugLine> history;
} console;

void do_debug(gui::Layout *layout, gui::Font *font)
{
    if (!gui::state.show_debug_console) {
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
        DebugLine *line = &console.history[i];

        glColor4f(line->r, line->g, line->b, 1.0f);
        gui::draw_text(font, line->line.c_str(), CONSOLE_MARGIN, text_begin, CONSOLE_TEXT_SIZE);


        // Move up and add a gap between lines.
        text_begin -= CONSOLE_TEXT_SIZE + CONSOLE_PADDING;

        if (text_begin < 0)
            break;
    }
}


void do_help(gui::Layout* layout, gui::Font* font) {
	if (!gui::state.show_help_screen) {
		return;
	}


	gui::do_solid_rect(
		layout,
		HELP_WIDTH+1000,
		HELP_HEIGHT,
		CONSOLE_BACKGROUND_SHADE,
		CONSOLE_BACKGROUND_SHADE,
		CONSOLE_BACKGROUND_SHADE
	);

	// Begin the text half a character above the bottom of debug.
	int text_begin = WINDOW_HEIGHT - HELP_HEIGHT + 30 + 2.5*CONSOLE_TEXT_SIZE;

    // Draw the buffer line first then add a gap between this line and the next.
    glColor4f(console.buffer.r, console.buffer.g, console.buffer.b, 1.0f);
    gui::draw_text(font, "Open Console - D", 450, text_begin, CONSOLE_TEXT_SIZE);
    gui::draw_text(font, "Close Console - Esc", 450, text_begin+100, CONSOLE_TEXT_SIZE);
    gui::draw_text(font, "Open Help Menu - H", 450, text_begin+200, CONSOLE_TEXT_SIZE);
    gui::draw_text(font, "Switch Video Feeds - []", 950, text_begin, CONSOLE_TEXT_SIZE);
    gui::draw_text(font, "Expand Video Feed - []", 950, text_begin+100, CONSOLE_TEXT_SIZE);
    gui::draw_text(font, "Close Help Menu - J", 950, text_begin+200, CONSOLE_TEXT_SIZE);
}

void log(std::string text, float r, float g, float b) {
    if(text.size() < 200){
        DebugLine line{text, r, g, b};
        console.history.push_back(line);
        return;
    }

    while(text.size() >= 200){
        for(int i = 200; i > 0; i--){
            if(text[i] == ' '){
                std::string l = text.substr(0,i);
                DebugLine line{l, r, g, b};
                console.history.push_back(line);
                text = text.substr(i, text.size());
                break;
            }
        }
    }
}

void handle_input(char c) {
	if (c == '\n') {
		console.history.push_back(console.buffer);
		console.buffer = {CONSOLE_PROMPT, 0, 1, 0};
	} else {
		console.buffer.line.push_back(c);
	}
}

void handle_keypress(int key, int mods) {
	if (key == GLFW_KEY_ENTER) {
		std::string command = console.buffer.line.substr(CONSOLE_PROMPT.size());

		console.history.push_back(console.buffer);
		console.buffer = {CONSOLE_PROMPT, 0, 1, 0};

		if (command == "test") {
			log("This is some red text.", 1, 0, 0);
		}
	} else if (key == GLFW_KEY_BACKSPACE) {
		if (mods & GLFW_MOD_CONTROL) {
			console.buffer = {CONSOLE_PROMPT, 0, 1, 0};
		} else {
			int real_len = console.buffer.line.size() - CONSOLE_PROMPT.size();
			if (real_len > 0) {
				console.buffer.line.pop_back();
			}
		}
	} else if (key == GLFW_KEY_ESCAPE) {
		gui::state.input_state = gui::InputState::KEY_COMMAND;
		gui::state.show_debug_console = false;

    } else if (key == GLFW_KEY_J) {
        gui::state.input_state = gui::InputState::KEY_COMMAND;
        gui::state.show_help_screen = false;
	}
}

}} // namespace gui::debug_console
