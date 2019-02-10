#include "gui.hpp"

#include <SDL.h>
#include <GL/gl.h>

#include <cstdio>
#include <cstdlib>
#include <cstdint>

#include <stack>


void Layout::reset_x() {
    LayoutState state = state_stack[state_stack_len - 1];

    current_x = state.x;
}

void Layout::reset_y() {
    LayoutState state = state_stack[state_stack_len - 1];

    current_y = state.y;
}

void Layout::push() { state_stack[state_stack_len++] = {current_x, current_y}; }

void Layout::pop() { state_stack_len--; }

void Layout::advance_x(int d) { current_x += d; }

void Layout::advance_y(int d) { current_y += d; }

void draw_solid_rect(float x, float y, float w, float h, float r, float g,
                     float b) {
    glBegin(GL_QUADS);

    glColor4f(r, g, b, 1.0f);

    glVertex2f(x, y);
    glVertex2f(x + w, y);
    glVertex2f(x + w, y + h);
    glVertex2f(x, y + h);

    glEnd();
}

void draw_textured_rect(float x, float y, float w, float h,
                        unsigned int texture_id) {
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, texture_id);
    glBegin(GL_QUADS);

    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);

    glTexCoord2f(0, 0);
    glVertex2f(x, y);
    glTexCoord2f(1, 0);
    glVertex2f(x + w, y);
    glTexCoord2f(1, 1);
    glVertex2f(x + w, y + h);
    glTexCoord2f(0, 1);
    glVertex2f(x, y + h);

    glEnd();
    glDisable(GL_TEXTURE_2D);
}

void do_solid_rect(Layout* layout, int width, int height, float r, float g,
                   float b) {
    int x = layout->current_x;
    int y = layout->current_y;

    draw_solid_rect(x, y, width, height, r, g, b);

    layout->advance_x(width);
    layout->advance_y(height);
}

void do_textured_rect(Layout* layout, int width, int height,
                      unsigned int texture_id) {
    int x = layout->current_x;
    int y = layout->current_y;

    draw_textured_rect(x, y, width, height, texture_id);

    layout->advance_x(width);
    layout->advance_y(height);
}
