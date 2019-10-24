#ifndef TEXT_H
#define TEXT_H

#include <GL/gl.h>
#include <SDL.h>

#include "stb_image.h"
#include "stb_truetype.h"
// Represents a font.
struct Font {
    // Information that keeps track of each character that we want to be able to draw.
    stbtt_bakedchar baked_chars[95];

    // The font is one big texture!
    unsigned int texture_id;
};

// Loads a font from a TTF file to our internal font representation.
void load_font(Font* font, const char* file_name);

// Returns the width and height of the given text if it were rendered with the given font.
void size_text(Font* font, const char* text, int* width, int* height);

void draw_text(Font* font, const char* text, int x, int y, float height);

#endif
