#ifndef GUI_HPP
#define GUI_HPP

#include "../network/network.hpp"
#include "session.hpp"
#include "camera_feed.hpp"

#include <vector>
#include <unordered_map>
#include <string>
#include <stdlib.h>
#include <GLFW/glfw3.h>

namespace gui {

const int WINDOW_WIDTH = 1920;
const int WINDOW_HEIGHT = 1080;

const int LOG_VIEW_WIDTH = 585;
const int LOG_VIEW_HEIGHT = 440;

const int PRIMARY_FEED_WIDTH = 1280;
const int PRIMARY_FEED_HEIGHT = 720;

const int SECONDARY_FEED_WIDTH = 550;
const int SECONDARY_FEED_HEIGHT = 310;

enum class InputState {
    KEY_COMMAND,
    DEBUG_CONSOLE,
    STOPWATCH_MENU,

    CAMERA_MATRIX,
    CAMERA_MOVE,
    CAMERA_MOVE_TARGET,

    AUTONOMY_CONTROL,
    AUTONOMY_EDIT_TARGET,
};

struct Font {
    // Information that keeps track of each character that we want to be able to draw.
    stbtt_bakedchar baked_chars[95];

    // The font is one big texture!
    unsigned int texture_id;

    // Maximum height of ASCII characters at the loaded size.
    int max_height;
};

// Stores the global GUI state.
// Anything that needs to be persistent goes in here.
struct GlobalState {
    GLFWwindow* window;

    Font global_font;

    InputState input_state = InputState::KEY_COMMAND;

    bool show_debug_console = false;
};

// This is how you access the global state.
extern GlobalState state;

// Loads a font from a TTF file to our internal font representation.
// The size here is the maximum size at which the font should be rendered.
// Returns true if the font was loaded and false otherwise.
bool load_font(Font* font, const char* file_name, int size);

// Returns the width of the given string if it were rendered with the given
// font at the given height.
int text_width(Font* font, const char* text, int height);

// Returns the index of the last character that can fit on a line of the given width.
int text_last_index_that_can_fit(Font* font, const char* text, float width, int height);

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
        current_x = state_stack[state_stack_len - 1].x;
        current_y = state_stack[state_stack_len - 1].y;

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

//Grabs the current texture of the camera feed render
unsigned char* get_current_camera_frame(unsigned int texture_id, int texture_size);

//Saves the current camera feed frame as a bitmap image and returns the name of the file
char * gen_bitmap_from_camera_feed(Session* bs_session);

// Loads a png or jpeg image into memory, and returns an OpenGL texture id.
unsigned int load_texture(const char* file_name);

// Loads a png image into memory, specifically with an alpha channel.
unsigned int load_texture_alpha(const char* file_name);

// Renders a rectangle with the given texture at the given screen coords.
void fill_textured_rect(int x, int y, int w, int h, unsigned int texture_id);

// Renders a rectangle with the given texture at the given screen coords.
// The texture will be blended with the current OpenGL color.
void fill_textured_rect_mix_color(int x, int y, int w, int h, unsigned int texture_id);

// Fills the specified rectangle with the currently-set OpenGL color.
void fill_rectangle(int x, int y, int w, int h);

//Renders a circle around a given x,y coordinate. 
void do_circle(int x, int y, int radius);

//Sets color of stopwatch based on its current state
void set_stopwatch_icon_color(Session *bs_session);

//Displays the time currently contained in stopwatch
const char* get_stopwatch_text(Session *bs_session);

//Displays info about rover connection and controller mode
void do_info_panel(Layout* layout, int width, int height, Session *bs_session);

void do_subsystem_panel(Layout* layout, int width, int height, Session *bs_session);

//Sets up the window to display stopwatch information
void do_stopwatch_menu(Session *bs_session);

//Sets up the help menu window
void do_help_menu(std::vector<const char*> commands, std::vector<const char*> debug_commands, Session *bs_session);

//Displays the two images captured for science testing side-by-side in a window
void do_image_display(unsigned int first_image_texture_id, unsigned int second_image_texture_id, Session *bs_session);

//Draws info acquired by the lidar system
void do_lidar(Layout* layout, Session *bs_session);

//Deals with moving the camera to different parts of the UI
void do_camera_move_target(Session *bs_session);

//Deals with camera matrix
void do_camera_matrix(Session *bs_session);

//Displays info during autonomous navigation
void do_autonomy_control(Session *bs_session);

//update general mode GUI layout
void do_general_gui(Session *bs_session);

//update drive mode GUI layout
void do_drive_gui(Session *bs_session);

//Draws the Arm mode GUI in full
void do_arm_gui(Session *bs_session);

//Draws the Science mode GUI in full
void do_science_gui(Session *bs_session);

//Draws the Autonomy mode GUI in full
void do_autonomy_gui(Session *bs_session);

//Draws the GUI in full
void do_gui(Session *bs_session);

void glfw_character_callback(GLFWwindow* window, unsigned int codepoint);

void glfw_key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);

//Commands for pressing keys in specific focus modes
void universal_key_commands(Session* bs_session, GLFWwindow* window, int key, int action, int mods);
void general_key_commands(Session* bs_session, GLFWwindow* window, int key, int action, int mods);
void arm_key_commands(Session* bs_session, GLFWwindow* window, int key, int action, int mods);
void drive_key_commands(Session* bs_session, GLFWwindow* window, int key, int action, int mods);
void science_key_commands(Session* bs_session, GLFWwindow* window, int key, int action, int mods);
void autonomy_key_commands(Session* bs_session, GLFWwindow* window, int key, int action, int mods);
void neutral_key_commands(Session* bs_session, GLFWwindow* window, int key, int action, int mods);

} // namespace gui

#endif
