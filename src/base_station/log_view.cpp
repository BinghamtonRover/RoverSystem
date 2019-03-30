#include "log_view.hpp"

#include <vector>

namespace gui {
namespace log_view {

const int MARGIN = 2;
const int PADDING = 5;
const int FONT_SIZE = 15;

unsigned int chars_per_line = 32;
unsigned int num_lines = 15;
unsigned int total_lines = 1000;
unsigned int view_index = 0;

// List of messages the log displays
std::vector<std::string> logMessages;
// Four parallel lists that are also parallel to logMessages
std::vector<float> red, green, blue, alpha;

// Helper method for addMessage
static void removeOldMessages()
{
    while (logMessages.size() > total_lines) {
        logMessages.erase(logMessages.begin());
        red.erase(red.begin());
        green.erase(green.begin());
        blue.erase(blue.begin());
        alpha.erase(alpha.begin());
    }
}

void calc_sizing(gui::Font* font, int width, int height) {
	chars_per_line = width / gui::text_width(font, "m", FONT_SIZE);
	num_lines = (height + PADDING) / (FONT_SIZE + PADDING);
}

void print(std::string m, float r, float g, float b, float a)
{
    while (m.length() > 0) {
        if (m.length() <= chars_per_line) {
            logMessages.push_back(m);
            red.push_back(r);
            green.push_back(g);
            blue.push_back(b);
            alpha.push_back(a);

            break;
        } else {
            logMessages.push_back(m.substr(0, chars_per_line));
            red.push_back(r);
            green.push_back(g);
            blue.push_back(b);
            alpha.push_back(a);
	
            m.erase(0, num_lines);
        }
    }
    
    if(logMessages.size() >= num_lines) {
	view_index++;
    }

    removeOldMessages();
}

void moveUpOne() {
	if(logMessages.size() >= num_lines and view_index > 0) {
		view_index--;
	}
}

void moveDownOne() {
	if(logMessages.size() >= num_lines and view_index < logMessages.size() - num_lines) {
		view_index++;
	}
}

void moveTop() {
	if(logMessages.size() >= num_lines) {
		view_index = 0;
	}
}

void moveBottom() {
	if(logMessages.size() >= num_lines) {
		view_index = logMessages.size() - num_lines;
	}
}

void testLog(unsigned int num) {
	for(unsigned int i = 0; i < num; i++) {
		print("This is a test message. #" + std::to_string(i+1), 255, 255, 255, 255);
		if(logMessages.size() >= num_lines) {
			view_index++;
    		}
	}
}

void do_log(gui::Layout* layout, int width, int height, gui::Font* font) {
	int x = layout->current_x;
	int y = layout->current_y;

        gui::do_solid_rect(layout, width, height, 0, 0, 0);

	// We want to print a num_lines amount of times *unless* logMessages is too small
	unsigned int nick_pellegrino_is_cool = logMessages.size();
	if(logMessages.size() >= num_lines) {
		nick_pellegrino_is_cool = num_lines;
	}

	// Extra Failsafe
	if(logMessages.size() < num_lines + view_index) {
		view_index = 0;
	}	

	for (unsigned int i = view_index; i < nick_pellegrino_is_cool + view_index; i++) {
		const char* str = logMessages[i].c_str();

		glColor4f(red[i], green[i], blue[i], alpha[i]);

		gui::draw_text(font, str, x + MARGIN, y + MARGIN + (i - view_index)*(FONT_SIZE + 5), FONT_SIZE);
	}
}

} } // namespace gui::log_view
