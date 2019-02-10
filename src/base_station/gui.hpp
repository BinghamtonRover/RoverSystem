#ifndef GUI_H
#define GUI_H

// We know that our base station will have this resolution.
const int WINDOW_WIDTH = 1920;
const int WINDOW_HEIGHT = 1080;

// Send movement updates 3x per second.
const int MOVMENT_SEND_INTERVAL = 1000 / 3;

// Debug Key bind
const char DEBUG_KEY = 'd';

struct LayoutState {
    int x = 0;
    int y = 0;
};

struct Layout {
    LayoutState state_stack[10];
    int state_stack_len = 0;

    int current_x;
    int current_y;

    void reset_x();
    void reset_y();
    void push();
    void pop();
    void advance_x(int d);
    void advance_y(int d);
};

void draw_solid_rect(float x, float y, float w, float h, float r, float g,
                     float b);

void draw_textured_rect(float x, float y, float w, float h,
                        unsigned int texture_id);

void do_solid_rect(Layout* layout, int width, int height, float r, float g,
                   float b);

void do_textured_rect(Layout* layout, int width, int height,
                      unsigned int texture_id);

#endif
