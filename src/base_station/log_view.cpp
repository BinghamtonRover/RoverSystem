#include "log_view.hpp"

#include <vector>

///********** log_view.cpp *********************************************************
/// Holds where in the logMessages you are looking, with 0 being the top line | 20
/// True when user is at the bottom of the messages | 23
///*********************************************************************************

namespace gui {
namespace log_view {

const int MARGIN = 2;
const int PADDING = 5;
const int FONT_SIZE = 15;

unsigned int chars_per_line = 32;
unsigned int num_lines = 15;
unsigned int total_lines = 1000;

// >> Holds where in the logMessages you are looking, with 0 being the top line
unsigned int view_index = 0;

// >> True when user is at the bottom of the messages
bool lockBottom = true;

// List of messages the log displays
std::vector<std::string> logMessages;
// Four parallel lists that are also parallel to logMessages
std::vector<float> red, green, blue, alpha;

// Helper method for addMessage
static void removeOldMessages() {
    while (logMessages.size() > total_lines) {
        logMessages.erase(logMessages.begin());
        red.erase(red.begin());
        green.erase(green.begin());
        blue.erase(blue.begin());
        alpha.erase(alpha.begin());
    }
}

void print(Font* font, int width, std::string m, float r, float g, float b, float a) {
    while (m.length() > 0) {
        int idx = gui::text_last_index_that_can_fit(font, m.c_str(), width, FONT_SIZE);

        logMessages.push_back(m.substr(0, idx + 1));
        red.push_back(r);
        green.push_back(g);
        blue.push_back(b);
        alpha.push_back(a);

        m.erase(0, idx + 1);
    }

    removeOldMessages();
}

void moveUpOne() {
    if (logMessages.size() >= num_lines && view_index > 0) {
        view_index--;
        lockBottom = false;
    }
}

void moveDownOne() {
    if (logMessages.size() >= num_lines && view_index < logMessages.size() - num_lines) {
        view_index++;
    }
    if (view_index == logMessages.size() - num_lines) {
        lockBottom = true;
    }
}

void moveTop() {
    if (logMessages.size() >= num_lines) {
        view_index = 0;
        lockBottom = false;
    }
}

void moveBottom() {
    if (logMessages.size() >= num_lines) {
        view_index = logMessages.size() - num_lines;
        lockBottom = true;
    }
}

void do_log(gui::Layout* layout, int width, int height, Font* font) {
    int x = layout->current_x;
    int y = layout->current_y;

    num_lines = height / (FONT_SIZE + 5);

    gui::do_solid_rect(layout, width, height, 0, 0, 0);

    // We want to print a num_lines amount of times *unless* logMessages is too small
    unsigned int tempSize = logMessages.size();
    if (logMessages.size() >= num_lines) {
        tempSize = num_lines;
    }

    // Extra Failsafe
    if (logMessages.size() < num_lines + view_index) {
        view_index = 0;
    }

    // Lock Bottom
    if (lockBottom) {
        moveBottom();
    }

    for (unsigned int i = view_index; i < tempSize + view_index; i++) {
        const char* str = logMessages[i].c_str();

        glColor4f(red[i], green[i], blue[i], alpha[i]);

        gui::draw_text(font, str, x + MARGIN, y + MARGIN + (i - view_index) * (FONT_SIZE + 5), FONT_SIZE);
    }
}

void log_view_handler(logger::Level level, std::string message) {
    float r, g, b, a = 1.0f;

    switch (level) {
        case logger::DEBUG:
            r = 0.70f;
            g = 0.70f;
            b = 0.70f;
            break;
        case logger::INFO:
            r = 1.0f;
            g = 1.0f;
            b = 1.0f;
            break;
        case logger::WARNING:
            r = 1.00f;
            g = 0.52f;
            b = 0.01f;
            break;
        case logger::ERROR:
            r = 1.0f;
            g = 0.0f;
            b = 0.0f;
            break;
    }

    time_t current_time;
    time(&current_time);
    struct tm* time_info = localtime(&current_time);

    char time_string_buffer[200];
    strftime(time_string_buffer, sizeof(time_string_buffer), "[%H:%M:%S] ", time_info);

    std::string full_string = std::string(time_string_buffer) + message;

    gui::log_view::print(&gui::state.global_font, LOG_VIEW_WIDTH, full_string, r, g, b, a);
}

} // namespace log_view
} // namespace gui
