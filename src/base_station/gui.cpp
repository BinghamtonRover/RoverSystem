#include <GL/gl.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "gui.hpp"

#define STB_TRUETYPE_IMPLEMENTATION
#include "stb_truetype.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

namespace gui {

void load_font(Font* font, const char* file_name, int size) {
    // Open the font file for reading.
    FILE* font_file = fopen(file_name, "r");

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
    //      1) the index of the font we want within the file (TTF files can contain multiple fonts). We want the first one.
    //      2) the pixel height of the font. Should be the max we ever want to render.
    //      3) the memory we allocated for the bitmap.
    //      4) the width of the bitmap.
    //      5) the height of the bitmap.
    //      6) the character code of the first character we want to support. We want to just render the visible ASCII characters,
    //         which start at 32.
    //      7) the number of characters we want to support. We want all from 32 until the second to last signed ASCII character.
    //      8) an array in which information about each character is stored (the location of the character in the bitmap).
    if (stbtt_BakeFontBitmap(font_buffer, 0, size, font_bitmap, 1024, 1024, 32, 127 - 32, font->baked_chars) < 0) {
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

    glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE_ALPHA, 1024, 1024, 0, GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE, real_font_bitmap);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	// Last thing: we need to set the max height.
	int max_height = 0;
	for (char c = 32; c <= 127; c++) {
		float x = 0, y = 0;
		stbtt_aligned_quad q;
		stbtt_GetBakedQuad(font->baked_chars, 1024, 1024, c - 32, &x, &y, &q, 1);

		if (q.y1 - q.y0 > max_height) {
			max_height = q.y1 - q.y0;
		}
	}

	font->max_height = max_height;
}

// Returns the width of the given string if it were rendered with the given
// font at the given height.
int text_width(Font* font, const char* text, int height) {
    // Basic idea: iterate through each character in text.
    // The rightmost x coordinate of the last character is the width of the whole string.

    float total_width = 0;
    {
        float x = 0, y = 0;
        for (const char* text_ptr = text; *text_ptr; text_ptr++) {
            stbtt_aligned_quad q;
            stbtt_GetBakedQuad(font->baked_chars, 1024, 1024, *text_ptr - 32, &x, &y, &q, 1);

            if (*(text_ptr + 1) == 0) {
                total_width = q.x1;
            }
        }
    }

	// We want to make sure our width is correctly reported.
	float scale = (float)height / (float)font->max_height;

	return (int) ((float)total_width * scale);
}

// Draws the string with the given font at the given height.
void draw_text(Font* font, const char* text, int x, int y, float height) {
    // We want the tallest character to have the height we specified.
    // That way, the string will be at most height pixels tall.
    // This works because scale will be multiplied by each character height.
    // When the tallest character comes along, we have max_height * (height / max_height)
    // which simplifies to height.
    float scale = height / font->max_height;

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
            glTexCoord2f(q.s0, q.t0); glVertex2f(q.x0, q.y0);
            glTexCoord2f(q.s1, q.t0); glVertex2f(q.x1, q.y0);
            glTexCoord2f(q.s1, q.t1); glVertex2f(q.x1, q.y1);
            glTexCoord2f(q.s0, q.t1); glVertex2f(q.x0, q.y1);

            text++;
        }
    }
    glEnd();
    glDisable(GL_TEXTURE_2D);

    // Restores the MODELVIEW matrix that was used prior.
    glPopMatrix();
}

// Fills a circle centered at the given coordinates with the given radius.
// The angular resolution indicates the number of vertices per radian to use.
// The default value is gui::ANGULAR_RES. If the resulting circle
// looks jagged at the edge, increase this value. When drawing a small circle,
// it is more efficient to lower the angular resolution.
void fill_circle(int center_x, int center_y, int radius, int angular_res) {
    // Here, we'll use the modelview matrix a bit.

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();

    // Translate to where we want our circle center to be, and scale by the radius.
    // Doing this allows us to use sin and cos alone.
    glTranslatef(center_x, center_y, 0.0f);
    glScalef(radius, radius, 1.0f);

    int NUM_VERTICES = (int) (2.0f * M_PI * angular_res);

    glBegin(GL_TRIANGLE_FAN);

    for (int i = 0; i <= NUM_VERTICES; i++) {
        float theta = i * (2.0f * M_PI / (float) NUM_VERTICES); // Get our progress around the circle, in radians.

        // Because we used the modelview matrix, all we need here is sin and cos.
        // This will give us points that are on the unit circle (radius 1).
        // But since we scaled by the radius, this will multiply and our circle will
        // have the requested radius! And the translation will cause our circle to be placed appropriately.
        glVertex2f(cosf(theta), sinf(theta));
    }

    glEnd();

    glPopMatrix();
}

// Fills a rectangle at the given coordinates (top left corner) with the given
// width and height.
void fill_rectangle(int x, int y, int w, int h) {
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();

	glTranslatef(x, y, 0.0f);
	glScalef(w, h, 1.0f);

	glBegin(GL_QUADS);

	glVertex2f(0, 0);
	glVertex2f(0, 1);
	glVertex2f(1, 1);
	glVertex2f(1, 0);

	glEnd();

	glPopMatrix();
}


void fill_textured_rect(int x, int y, int w, int h, unsigned int texture_id) {
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();

	glTranslatef(x, y, 0.0f);
	glScalef(w, h, 1.0f);

	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, texture_id);

	glColor4f(1.0f, 1.0f, 1.0f, 1.0f);

	glBegin(GL_QUADS);

	glTexCoord2f(0, 0); glVertex2f(0, 0);
	glTexCoord2f(0, 1); glVertex2f(0, 1);
	glTexCoord2f(1, 1); glVertex2f(1, 1);
	glTexCoord2f(1, 0); glVertex2f(1, 0);

	glEnd();

	glDisable(GL_TEXTURE_2D);

	glPopMatrix();
}

void do_solid_rect(Layout* layout, int width, int height, float r, float g, float b) {
    int x = layout->current_x;
    int y = layout->current_y;

	glColor4f(r, g, b, 1.0f);
    fill_rectangle(x, y, width, height);

    layout->advance_x(width);
    layout->advance_y(height);
}

void do_textured_rect(Layout* layout, int width, int height, unsigned int texture_id) {
    int x = layout->current_x;
    int y = layout->current_y;

    fill_textured_rect(x, y, width, height, texture_id);

    layout->advance_x(width);
    layout->advance_y(height);
}

unsigned int load_texture(const char* file_name) {
    int texture_w, texture_h, texture_channels;
    // This loads a PNG or JPEG file into memory as an array of RGB pixels.
    // We specify that we do not care about the alpha channel, whether it exists or not, by asking for 3 channels.
    // This function gives us the texture's size and how many channels it actually loaded.
    // The format of texture_data is what is frequently referred to as RGB24, where each pixel is composed
    // of three unsigned bytes: R, G, and B. Each color channel can have a value in [0, 255].
    unsigned char* texture_data = stbi_load(file_name, &texture_w, &texture_h, &texture_channels, 3);

    // We need to register this texture with OpenGL.
    // Once we do that, we need a way to refer to the texture when we want to draw it.
    // OpenGL does this with an unsigned int id.
    // The weird signature for glGenTextures() comes from the fact that it works with arrays.
    // For example, this would be valid:
    //     unsigned int texture_ids[15];
    //     glGenTextures(15, texture_ids);
    // In this case, we have one texture id, so we pretend its pointer is an array of size one.
    // This is C++, so a pointer to an int is indeed an array of size one.
    unsigned int tex_id;
    glGenTextures(1, &tex_id);

    // OpenGL has any functions which modify or draw a texture.
    // Before using them, we have to specify which texture should be modified.
    glBindTexture(GL_TEXTURE_2D, tex_id);

    // Textures have many adjustable parameters. These two must be set, otherwise the texture won't render.
    // This tells OpenGL how to scale the texture. The min filter specifies how to scale the texture
    // if it needs to be shrunk in order to be rendered. The mag filter specifies how to scale the texture
    // if it needs to be expanded in order to be rendered. The filter we use for both is the linear filter,
    // which does bilinear interpolation across the pixels of the texture. There is also GL_NEAREST, which
    // just finds the nearest texture pixel, which is faster but looks worse.
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    // This uploads the texture data to OpenGL's internal memory.
    // The first zero specifies the mipmap level we want to modify.
    // Mipmaps are a way to simplify textures when they are very small, in order to more efficiently render far-away objects.
    // If you've ever played a 3D video game, its related to Level of Detail (LOD).
    // We haven't enabled mipmapping (it can be enabled via glTexParameter(), but we don't need it for 2D), so
    // 0 specifies the default mipmap level.
    // The first GL_RGB specifies how we want OpenGL to represent the texture internally.
    // If we needed alpha, we could use GL_RGBA here. We don't need it though, because we didn't load our image
    // with alpha support so it would be a waste of memory.
    // That second zero is required to be zero. Don't ask.
    // The second GL_RGB specifies the encoding of our texture data. Then we specify that each color channel is
    // encoded in an unsigned byte. stbi_load() dictates these values.
    // Finally, we pass the raw texture data.
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texture_w, texture_h, 0, GL_RGB, GL_UNSIGNED_BYTE, texture_data);

    stbi_image_free(texture_data);

    return tex_id;
}
}
