#ifndef GUI_HPP
#define GUI_HPP

#include "stb_truetype.h"

#include <GLFW/glfw3.h>

namespace gui {

const int WINDOW_WIDTH = 1920;
const int WINDOW_HEIGHT = 1080;

enum class InputState {
	KEY_COMMAND,
	DEBUG_CONSOLE,
    HELP_SCREEN
};

// Stores the global GUI state.
// Anything that needs to be persistent goes in here.
struct GlobalState {
	GLFWwindow* window;

	InputState input_state = InputState::KEY_COMMAND;

	bool show_debug_console = false;
    bool show_help_screen = false;
};

// This is how you access the global state.
extern GlobalState state;

struct Font {
    // Information that keeps track of each character that we want to be able to draw.
    stbtt_bakedchar baked_chars[95];

    // The font is one big texture!
    unsigned int texture_id;

	// Maximum height of ASCII characters at the loaded size.
	int max_height;
};

// Loads a font from a TTF file to our internal font representation.
// The size here is the maximum size at which the font should be rendered.
// Returns true if the font was loaded and false otherwise.
bool load_font(Font* font, const char* file_name, int size);

// Returns the width of the given string if it were rendered with the given
// font at the given height.
int text_width(Font* font, const char* text, int height);

void draw_text(Font* font, const char* text, int x, int y, float height);

struct LayoutState {
    int x = 0;
    int y = 0;
};

struct Layout {
    LayoutState state_stack[10];
    int state_stack_len = 0;

    int current_x;
    int current_y;

    void reset_x() {
        LayoutState state = state_stack[state_stack_len - 1];

        current_x = state.x;
    }

    void reset_y() {
        LayoutState state = state_stack[state_stack_len - 1];

        current_y = state.y;
    }

    void push() {
        state_stack[state_stack_len++] = { current_x, current_y };
    }

    void pop() {
        state_stack_len--;
    }

    void advance_x(int d) {
        current_x += d;
    }

    void advance_y(int d) {
        current_y += d;
    }
};

// Renders a solid rectangle at the position determined by the given layout,
// filled with the given color.
void do_solid_rect(Layout* layout, int width, int height, float r, float g, float b);

// Renders a rectangle at the position determined by the given layout,
// with the given texture.
void do_textured_rect(Layout* layout, int width, int height, unsigned int texture_id);

// Loads a png or jpeg image into memory, and returns an OpenGL texture id.
unsigned int load_texture(const char* file_name);
}

#endif
