#ifndef DEBUG_H
#define DEBUG_H

#include "text.hpp"

#include <vector>
#include <string>

#define LINE_WRAP 150

struct DebugConsole {

    std::string prompt;
    Font font;

    struct Color {
        float r = 0.0f;
        float g = 1.0f;
        float b = 0.0f;
        float a = 1.0f;
        Color(){}
    };

    struct DebugLine {
        std::string line;
        Color color;

        DebugLine(){}
        DebugLine(const char* s): line{s}{}
        DebugLine(const std::string& s): line{s}{}
        DebugLine(const std::string& s, Color c): line{s}, color{c}{}
    };

    DebugLine buffer;
    std::vector<DebugLine> history;

    DebugConsole(std::string font_file, std::string p=">");
    void draw_text_wrapper(Font* font, const char* text, int x, int y, float height);
    void do_debug();

    int process(std::string command);
    void log(const std::string& text, float r, float g, float b);
    void log(const std::string& text);
};

#endif
