#include "text.hpp"

#include <GL/gl.h>
#include <SDL.h>
// stb_image provides basic image loading.
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

// stb_truetype provides basic font rendering.
#define STB_TRUETYPE_IMPLEMENTATION
#include "stb_truetype.h"

// Loads a font from a TTF file to our internal font representation.
void load_font(Font *font, const char *file_name)
{
    // Open the font file for reading.
    FILE *font_file = fopen(file_name, "r");

    // We need some space to store our font. Let's just try this much.
    // This could be dynamically allocated to be the size of the file, but I'm lazy.
    // This is static so that it is not allocated on the stack.
    static unsigned char font_buffer[1 << 20];

    // Read the whole file.
    fread(font_buffer, 1, 1 << 20, font_file);

    fclose(font_file);

    // Make some space for the font bitmap texture data.
    // The whole font is stored in one texture, with all the characters we want to draw
    // packed in a square texture. It looks something like this:
    //     a b c d e f g h i j k l m n o p q r s t u v w x y z
    //     A B C D E F G H I J K L M N O P Q R S T U V W X Y Z
    //     1 2 3 4 5 6 7 8 9 0 ! @ # $ % ^ & * ( ) - _ + = / ? ...
    // We don't know how big the bitmap needs to be.
    // A better strategy would be to guess, try to bake the bitmap, and if it fails, reallocate and retry.
    // But I'm lazy and this is a basic example, so I go with this approach.
    static unsigned char font_bitmap[1024 * 1024];

    // This bakes the bitmap. Each pixel in the bitmap is represented by a single unsigned byte.
    // 0 indicates nothing, and 255 indicates a "lit" pixel with alpha = 1.
    // Everything in between indicates a pixels with alpha != 1. This is used at the edges of characters
    // for anti-aliasing.
    // Explanation of parameters:
    //      0) the raw font file contents.
    //      1) the index of the font we want within the file (TTF files can contain multiple fonts). We want the first
    //      one. 2) the pixel height of the font. This is hardcoded! We want to pick something reasonably large so that
    //      we never
    //         have to scale it up (that would make the text look pixelated). In order to have perfect rendering at
    //         every text size, we would have to use something called signed distance fields, which are really cool but
    //         would require using OpenGL 4, which is overcomplicated for our purposes.
    //      3) the memory we allocated for the bitmap.
    //      4) the width of the bitmap.
    //      5) the height of the bitmap.
    //      6) the character code of the first character we want to support. We want to just render the visible ASCII
    //      characters,
    //         which start at 32.
    //      7) the number of characters we want to support. We want all from 32 until the second to last signed ASCII
    //      character. 8) an array in which information about each character is stored (the location of the character in
    //      the bitmap).
    if (stbtt_BakeFontBitmap(font_buffer, 0, 100, font_bitmap, 1024, 1024, 32, 127 - 32, font->baked_chars) < 0) {
        fprintf(stderr, "[!] Failed to load font bitmap!\n");
        exit(1);
    }

    // Unfortunately, OpenGL is incapable of rendering the above bitmap (it doesn't recognize the format).
    // So we need to convert it to something it does understand.
    // We will be converting the bitmap to (luminance, alpha) pixels.
    // Luminance is copied to the R, G, and B channels, and alpha is preserved.
    static unsigned char real_font_bitmap[1024 * 1024 * 2];

    // For each pixel in the bitmap, set the luminance to full (255).
    // Then set the alpha to the value of the pixel from font_bitmap.
    // Effectively, font_bitmap tells us what the alpha should be for each pixel in each character.
    // OpenGL doesn't have a texture type for this (where all R, G, and B channels are set to 255),
    // so that's why we have to convert it.
    for (int y = 0; y < 1024; y++) {
        for (int x = 0; x < 1024; x++) {
            real_font_bitmap[(y * 1024 + x) * 2 + 0] = 255;
            real_font_bitmap[(y * 1024 + x) * 2 + 1] = font_bitmap[y * 1024 + x];
        }
    }

    glGenTextures(1, &(font->texture_id));

    glBindTexture(GL_TEXTURE_2D, font->texture_id);

    glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE_ALPHA, 1024, 1024, 0, GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE,
                 real_font_bitmap);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
}

// Returns the width and height of the given text if it were rendered with the given font.
void size_text(Font *font, const char *text, int *width, int *height)
{
    // Basic idea: iterate through each character in text, and check its height.
    // Check to see if it has the greatest height of all characters we want to display.
    // Then the rightmost x coordinate of the last character is the width of the whole string.

    float max_height = 0;
    float total_width = 0;
    {
        float x = 0, y = 0;
        for (const char *text_ptr = text; *text_ptr; text_ptr++) {
            stbtt_aligned_quad q;
            stbtt_GetBakedQuad(font->baked_chars, 1024, 1024, *text_ptr - 32, &x, &y, &q, 1);

            if (q.y1 - q.y0 > max_height) {
                max_height = q.y1 - q.y0;
            }

            if (*(text_ptr + 1) == 0) {
                total_width = q.x1;
            }
        }
    }

    *width = (int)total_width;
    *height = (int)max_height;
}

void draw_text(Font *font, const char *text, int x, int y, float height)
{
    // Scale the text based on height. removed max height calculation which was causing the
    // text size to change while typing. right now this uses a magic number, should be changed
    float scale = height / 85;

    // Matrix and texture stuff must be called before glBegin()!

    // We use the modelview matrix for our scale.
    // We could just multiply all our coordinates by scale before passing
    // to OpenGL, but that would be super verbose.
    // This will do the multiplication for us!
    // The 1.0f at the end is for the z axis, which is unused since we are doing 2D.
    // glPushMatrix() saves whatever the matrix looked like before so we can restore it after.
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glTranslatef(x, y + height, 0.0f);
    glScalef(scale, scale, 1.0f);

    glBindTexture(GL_TEXTURE_2D, font->texture_id);

    glEnable(GL_TEXTURE_2D);
    glBegin(GL_QUADS);
    {
        float tx = 0, ty = 0;

        while (*text) {
            stbtt_aligned_quad q;

            stbtt_GetBakedQuad(font->baked_chars, 1024, 1024, *text - 32, &tx, &ty, &q, 1);
            glTexCoord2f(q.s0, q.t0);
            glVertex2f(q.x0, q.y0);
            glTexCoord2f(q.s1, q.t0);
            glVertex2f(q.x1, q.y0);
            glTexCoord2f(q.s1, q.t1);
            glVertex2f(q.x1, q.y1);
            glTexCoord2f(q.s0, q.t1);
            glVertex2f(q.x0, q.y1);

            text++;
        }
    }
    glEnd();
    glDisable(GL_TEXTURE_2D);

    // Restores the MODELVIEW matrix that was used prior.
    glPopMatrix();
}
