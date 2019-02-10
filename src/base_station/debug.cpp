#include "debug.hpp"
#include "text.hpp"
#include "gui.hpp"

#include <SDL.h>
#include <GL/gl.h>

#include <cstdio>
#include <cstdlib>
#include <cstdint>

#include <stack>
#include <string>
#include <vector>
#include <iostream>


DebugConsole::DebugConsole(std::string font_file, std::string p){
    load_font(&(this->font), "fira-mono.regular.ttf");
    prompt = p;
    buffer = p;
}

int DebugConsole::process(std::string command){
    if(command == "test1"){
        log("neat test!", 1.0, 0.0, 0.0);
        log("heres a log with normal colors!");
        return 0;
    }
    if(command == "longboi"){
        log("aaaaaaaaaaaaaaaaa aaaaaaaaaaaaaaaaaaaaaaaaaa aaaaaaaaaaaaaaaaaaaaa aaaaaaaaaaaaaaa"
            "bbbbbbbbbbbbbbbbb bbbbbbbbbbbbbbbbbbbbbbbbbb bbbbbbbbbbbbbbbbbbbbb bbbbbbbbbbbbbbb"   
            "ccccccccccccccccc cccccccccccccccccccccccccc ccccccccccccccccccccc ccccccccccccccc"   
            "ddddddddddddddddd dddddddddddddddddddddddddd ddddddddddddddddddddd ddddddddddddddd"   
            "eeeeeeeeeeeeeeeee eeeeeeeeeeeeeeeeeeeeeeeeee eeeeeeeeeeeeeeeeeeeee eeeeeeeeeeeeeee"   
            "fffffffffffffffff ffffffffffffffffffffffffff fffffffffffffffffffff fffffffffffffff"   
            "ggggggggggggggggg gggggggggggggggggggggggggg ggggggggggggggggggggg ggggggggggggggg"   
            "hhhhhhhhhhhhhhhhh hhhhhhhhhhhhhhhhhhhhhhhhhh hhhhhhhhhhhhhhhhhhhhh hhhhhhhhhhhhhhh"   
            "iiiiiiiiiiiiiiiii iiiiiiiiiiiiiiiiiiiiiiiiii iiiiiiiiiiiiiiiiiiiii iiiiiiiiiiiiiii"   
            "jjjjjjjjjjjjjjjjj jjjjjjjjjjjjjjjjjjjjjjjjjj jjjjjjjjjjjjjjjjjjjjj jjjjjjjjjjjjjjj"   
            "kkkkkkkkkkkkkkkkk kkkkkkkkkkkkkkkkkkkkkkkkkk kkkkkkkkkkkkkkkkkkkkk kkkkkkkkkkkkkkk"   
            "lllllllllllllllll llllllllllllllllllllllllll lllllllllllllllllllll lllllllllllllll"   
            "mmmmmmmmmmmmmmmmm mmmmmmmmmmmmmmmmmmmmmmmmmm mmmmmmmmmmmmmmmmmmmmm mmmmmmmmmmmmmmm"   
            , 0.0, 0.0, 1.0);
        return 0;
    }
    return 1;
}

void DebugConsole::log(const std::string& text, float r, float g, float b){
    std::string copy = text;
    Color c;
    c.r = r;
    c.g = g;
    c.b = b;

    if(text.size() < 200){
        DebugLine line(text, c);
        history.push_back(line);
        return;
    }

    while(copy.size() >= 200){
        for(int i = 200; i > 0; i--){
            if(copy[i] == ' '){
                std::string l = copy.substr(0,i);
                DebugLine line(l, c);
                history.push_back(line);
                copy = copy.substr(i, copy.size());
                break;
            }
        }
    }
}

void DebugConsole::log(const std::string& text){
    Color c;
    log(text, c.r, c.g, c.b);
}

void DebugConsole::do_debug(){
    int text_size = 16;

    glBegin(GL_QUADS);

    glColor4f(40.0f / 255.0f, 40.0f / 255.0f, 40.0f / 255.0f, 1.0f);
    glVertex2f(0, 0);
    glVertex2f(0, WINDOW_HEIGHT/5);
    glVertex2f(WINDOW_WIDTH, WINDOW_HEIGHT/5);
    glVertex2f(WINDOW_WIDTH, 0);

    glEnd();

    //int w = 0;
    //int h = 0;
    //size_text(&font, "abcdefghijklmnopqrstuvwxyz"
        //"ABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890!@#$%^&*()/.,?><`~';:\"\'", &w, &h);
    //std::cout << w << " " << h << '\n';

    //Begin the text half a character above the bottom of debug
    int text_begin = WINDOW_HEIGHT/5-1.5*text_size;

    //Draw the buffer line first then add a gap between this line and the next
    glColor4f(buffer.color.r, buffer.color.g, buffer.color.b, buffer.color.a);
    draw_text(&font, buffer.line.c_str(), 5, text_begin, text_size);
    text_begin-=text_size+2;

    //Write the history to the console until out of history or top of the screen is reached
    for(int i = history.size()-1; i >=0; i--){
        DebugConsole::Color col = history[i].color;
        glColor4f(col.r, col.g, col.b, col.a);
        draw_text(&font, history[i].line.c_str(), 5, text_begin, text_size);
        //move up and add a gap between lines
        text_begin-=text_size+2;
        if(text_begin < 0) break;
    }
}




