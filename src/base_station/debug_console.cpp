#include "debug_console.hpp"
#include "gui.hpp"
#include "log_view.hpp"
#include "waypoint.hpp"
#include <GLFW/glfw3.h>
#include "session.hpp"

#include <string>
#include <vector>
#include <stdexcept>

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

//Sets the callback for the console
void set_callback(CommandCallback callback) {
    console.callback = callback;
}

//Takes a string and returns a vector containing substrings that were separated by spaces
std::vector<std::string> split_by_spaces(std::string s) {
    std::vector<std::string> strings;

    size_t last = 0;
    size_t next = 0;

    //Loops through the string and adds the substrings to the vector
    while ((next = s.find(" ", last)) != std::string::npos) {
        strings.push_back(s.substr(last, next - last));
        last = next + 1;
    }

    strings.push_back(s.substr(last));

    return strings;
}

//Parses commands
void command_callback(std::string command, Session *bs_session) {
    auto parts = gui::debug_console::split_by_spaces(command);
    //Checks for test command
    if (parts[0] == "test") {
        //Executes commands if there are no other parameters (other than the command itself)
        if (parts.size() == 1) {
            log("This is some red text.", 1, 0, 0);
        } else {
            log("Invalid Input: \"test\" expects no parameters.", 1, 0, 0);
        }
    } 
    //Checks for the toggle debug command
    else if (parts[0] == "tdl") {
        //Ensures that there are no other parameters (other than the command itself)
        if (parts.size() == 1) {
            bool mode = logger::toggleDebugMode();
            if(mode) {
                log("DebugMode turned on", 1, 0, 1);
            } else {
                log("DebugMode turned off", 1, 0, 1);
            }
        } else {
            log("Invalid Input: \"tdl\" expects no parameters.", 1, 0, 0);
        }
    } 
    //Checks for the add waypoint command
    else if (parts[0] == "aw") {
        //Checks that there are 3 parameters
        if (parts.size() == 3) {
            try {
                float lat = std::stof(parts[1], nullptr);
                float lon = std::stof(parts[2], nullptr);
                waypoint::add_waypoint(lat,lon);
                log("Waypoint [" + std::to_string(lat) + ", " + std::to_string(lon) + "] added.", 1, 0, 1);
            } catch (const std::invalid_argument& ia) {
                log("Invalid Input: \"aw\" requires two doubles, but something else was provided. Example: \"aw 42.2 75.3\"", 1, 0, 0);
            } catch (const std::out_of_range& oor) {
                log("Invalid Input: a provided double was out of range", 1, 0, 0);
            }
        } else {
            log("Invalid Input: \"aw\" requires two doubles, for example: \"aw 42.2 75.3\"", 1, 0, 0);
        }
    } 
    //Checks for the list waypoints command
    else if (parts[0] == "lw") {
        //Ensures there is only one parameter (the command itself)
        if (parts.size() == 1) {
            log("Waypoints: ", 1, 0, 1);
            auto waypoints = waypoint::get_waypoints();
            for(unsigned int i = 0; i < waypoints.size(); i++) {
                std::string latStr = std::to_string(waypoints.at(i).latitude);
                std::string lonStr = std::to_string(waypoints.at(i).longitude);
                log("[" + latStr + ", " + lonStr + "]", 1, 1, 1);
            }
        } else {
            log("Invalid Input: \"lw\" expects no parameters.", 1, 0, 0);
        }
    } 
    //Checks for the help command
    else if (parts[0] == "help") {
        //Ensures there is only one parameter (other than the command itself)
        if (parts.size() == 1) {
            log("Press 'h' on the main screen for help.", 1, 1, 1);
        } else {
            log("Invalid Input: \"help\" expects no parameters.", 1, 0, 0);
        }
    }
    //Cehcks for the greyscale on command
    else if (parts[0] == "gs_on") {
        //Checks that there is only one parameter (other than the command itself)
        if (parts.size() == 1) {
            //Checks that there is a rover feed
            if (shared_feeds::bs_feed != NULL){
                //TODO: Fix this design hack, currently using shared_feeds to get the rover_feed established in main
                network::CameraControlMessage message = {
                    network::CameraControlMessage::Setting::GREYSCALE, // setting
                    //static_cast<uint8_t>(1), //setting
                    static_cast<bool>(true) //greyscale
                };
                network::publish(shared_feeds::bs_feed, &message);
            }
        } else {
            log("Invalid Input: \"gs_on\" expects no parameters.", 1, 0, 0);
        }
    }
    //Checks for the greyscale off command
    else if (parts[0] == "gs_off") {
        //Checks that there is one parameter (other than the command itself)
        if (parts.size() == 1) {
            if (shared_feeds::bs_feed != NULL){
                //TODO: Fix this design hack, currently using shared_feeds to get the rover_feed established in main
                network::CameraControlMessage message = {
                    network::CameraControlMessage::Setting::GREYSCALE, // setting
                    static_cast<bool>(false) //greyscale
                };
                network::publish(shared_feeds::bs_feed, &message);
            }
        }
        else {
            log("Invalid Input: \"gs_off\" expects no parameters.", 1, 0, 0);
        }
    }
    //Checks for the jpeg quality command
    else if (parts[0] == "jpeg_quality") {
        //Checks that there are two parameters
        if (parts.size() == 2) {
            try {
                int value = std::stoi(parts[1], nullptr);
                //Checks that the value is in the correct range
                if (value >= 0 && value <= 100) {
                    //TODO: Fix this design hack, currently using shared_feeds to get the rover_feed established in main
                    if (shared_feeds::bs_feed != NULL){
                        network::CameraControlMessage message = {
                            network::CameraControlMessage::Setting::JPEG_QUALITY, // setting
                            static_cast<uint8_t>(value) //jpegQuality
                        };
                        network::publish(shared_feeds::bs_feed, &message);
                    }
                } else {
                    log("Invalid Input: \"jpeg_quality\" requires an integer between 0 and 100, for example: \"jpeg_quality 30\"",1,0,0);
                }
            } catch (const std::logic_error& ia) {
                log("Invalid Input: \"jpeg_quality\" requires an integer between 0 and 100, but the provided value wasn't a valid integer. Example: \"jpeg_quality 30\"",1,0,0);
            }
        } else {
            log("Invalid Input: \"jpeg_quality\" requires an integer between 0 and 100, for example: \"jpeg_quality 30\"",1,0,0);
        }
    }
    //Checks for the move command
    else if (parts[0] == "move") {
        //Checks that there are 3 paramteres
        if (parts.size() == 3) {
            char left_direction_char = parts[1][0];
            int16_t left_speed = (int16_t) atoi(parts[1].substr(1).c_str());

            char right_direction_char = parts[2][0];
            int16_t right_speed = (int16_t) atoi(parts[2].substr(1).c_str());

            bs_session->last_movement_message.left = left_speed;
            bs_session->last_movement_message.right = right_speed;

            //Reverses direction of left message
            if (left_direction_char == 'b') {
                bs_session->last_movement_message.left *= -1;
            }

            //Reverses direction of right message
            if (right_direction_char == 'b') {
                bs_session->last_movement_message.right *= -1;
            }

            logger::log(
                logger::DEBUG,
                "> Update movement to %d, %d",
                bs_session->last_movement_message.left,
                bs_session->last_movement_message.right);
        } else {
            log("Invalid Input: \"move\" expects two integers.", 1, 0, 0);
        }
    }
    //Catches invalid input
    else if (parts[0].length() > 0) {
        log("Invalid Input: no matching command was found", 1, 0, 0);
    }
}

//Draws the debug console
void do_debug(gui::Layout* layout, Font* font) {
    //If the GUI isn't supposed to be drawn
    if (!gui::state.show_debug_console) {
        return;
    }

    //Exits the debug console
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

        // Breaks loop when there isn't room for the text
        if (text_begin < 0) break;
    }
}

// Draws text to the log
void log(std::string text, float r, float g, float b) {
    // If the text size is less than 200, add to the console history
    if (text.size() < 200) {
        DebugLine line{ text, r, g, b };
        console.history.push_back(line);
        return;
    }

    // If text is larger than 200, breaks it into two lines
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

// Pushes a character onto the buffer
void handle_input(char c) {
    console.buffer.line.push_back(c);
}

// Handles key presses in the console
void handle_keypress(int key, int mods, Session *session) {
    // If the enter key pressed, enters command to console
    if (key == GLFW_KEY_ENTER) {
        std::string command = console.buffer.line.substr(CONSOLE_PROMPT.size());

        console.history.push_back(console.buffer);
        console.buffer = { CONSOLE_PROMPT, 0, 1, 0 };
        
        console.callback(command, session);
        

    } 
    // If the backspace key is pressed, remove a character from the command buffer
    else if (key == GLFW_KEY_BACKSPACE) {
        if (mods & GLFW_MOD_CONTROL) {
            console.buffer = { CONSOLE_PROMPT, 0, 1, 0 };
        } else {
            int real_len = console.buffer.line.size() - CONSOLE_PROMPT.size();
            if (real_len > 0) {
                console.buffer.line.pop_back();
            }
        }
    } 
    //If the escape key is pressed, the console exits
    else if (key == GLFW_KEY_ESCAPE) {
        gui::state.input_state = gui::InputState::KEY_COMMAND;
        gui::state.show_debug_console = false;
    }
}
} // namespace debug_console
} // namespace gui
