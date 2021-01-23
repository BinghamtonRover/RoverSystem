#include "../network/network.hpp"
#include "waypoint.hpp"
#include "controller.hpp"
#include "camera_feed.hpp"
#include "waypoint_map.hpp"
#include "debug_console.hpp"
#include "log_view.hpp"

#include <GL/gl.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>

#include "gui.hpp"

#define STB_TRUETYPE_IMPLEMENTATION
#include "stb_truetype.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

namespace gui {

gui::GlobalState state;

bool load_font(Font* font, const char* file_name, int size) {
    // Open the font file for reading.
    FILE* font_file = fopen(file_name, "r");

    if (!font_file) {
        return false;
    }

    // We need some space to store our font. Let's just try this much.
    // This could be dynamically allocated to be the size of the file, but I'm lazy.
    // This is static so that it is not allocated on the stack.
    static unsigned char font_buffer[1 << 20];

    // Read the whole file.
    if (fread(font_buffer, 1, 1 << 20, font_file) == 0) {
        return false;
    }

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
    //      one. 2) the pixel height of the font. Should be the max we ever want to render. 3) the memory we allocated
    //      for the bitmap. 4) the width of the bitmap. 5) the height of the bitmap. 6) the character code of the first
    //      character we want to support. We want to just render the visible ASCII characters,
    //         which start at 32.
    //      7) the number of characters we want to support. We want all from 32 until the second to last signed ASCII
    //      character. 8) an array in which information about each character is stored (the location of the character in
    //      the bitmap).
    if (stbtt_BakeFontBitmap(font_buffer, 0, size, font_bitmap, 1024, 1024, 32, 127 - 32, font->baked_chars) < 0) {
        fprintf(stderr, "[!] Failed to load font bitmap!\n");
        return false;
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

    glTexImage2D(
        GL_TEXTURE_2D,
        0,
        GL_LUMINANCE_ALPHA,
        1024,
        1024,
        0,
        GL_LUMINANCE_ALPHA,
        GL_UNSIGNED_BYTE,
        real_font_bitmap);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    // Last thing: we need to set the max height.
    int max_height = 0;
    for (char c = 32; c < 127; c++) {
        float x = 0, y = 0;
        stbtt_aligned_quad q;
        stbtt_GetBakedQuad(font->baked_chars, 1024, 1024, c - 32, &x, &y, &q, 1);

        if (q.y1 - q.y0 > max_height) {
            max_height = q.y1 - q.y0;
        }
    }

    font->max_height = max_height;

    return true;
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
    float scale = (float) height / (float) font->max_height;

    return (int) ((float) total_width * scale);
}

// Returns the index of the last character that can fit on a line of the given width.
int text_last_index_that_can_fit(Font* font, const char* text, float width, int height) {
    // We want to make sure our width is correctly reported.
    float scale = (float) height / (float) font->max_height;

    float x = 0, y = 0;
    int i;
    for (i = 0; text[i]; i++) {
        stbtt_aligned_quad q;
        stbtt_GetBakedQuad(font->baked_chars, 1024, 1024, text[i] - 32, &x, &y, &q, 1);

        float line_width = q.x1 * scale;
        if (line_width > width) return i - 1;
    }

    return i - 1;
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

//Renders a circle around a given x,y coordinate.
void do_circle(int x, int y, int radius){
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glTranslatef(x,y,0.0f);
    int resolution = 600; //Determines how many points are being used to approximate the circle
    float theta = (2 * M_PI)/(float)resolution;
    float c = cosf(theta);
    float s = sinf(theta);
    float t;
    float px = radius;
    float py = 0;
    glLineWidth(1.0f);
    glBegin(GL_LINE_LOOP);
    for(int i = 0; i < resolution; i++){
        glVertex2f(px,py);
        t = px;
        px = c * px - s * py; //Rotating x component by theta
        py = s * t + c * py; //Rotating y component by theta
    }
    glEnd();
    glPopMatrix();

}

void fill_textured_rect_mix_color(int x, int y, int w, int h, unsigned int texture_id) {
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();

    glTranslatef(x, y, 0.0f);
    glScalef(w, h, 1.0f);

    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, texture_id);

    glBegin(GL_QUADS);

    glTexCoord2f(0, 0);
    glVertex2f(0, 0);
    glTexCoord2f(0, 1);
    glVertex2f(0, 1);
    glTexCoord2f(1, 1);
    glVertex2f(1, 1);
    glTexCoord2f(1, 0);
    glVertex2f(1, 0);

    glEnd();

    glDisable(GL_TEXTURE_2D);

    glPopMatrix();
}

void fill_textured_rect(int x, int y, int w, int h, unsigned int texture_id) {
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    fill_textured_rect_mix_color(x, y, w, h, texture_id);
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
    // Mipmaps are a way to simplify textures when they are very small, in order to more efficiently render far-away
    // objects. If you've ever played a 3D video game, its related to Level of Detail (LOD). We haven't enabled
    // mipmapping (it can be enabled via glTexParameter(), but we don't need it for 2D), so 0 specifies the default
    // mipmap level. The first GL_RGB specifies how we want OpenGL to represent the texture internally. If we needed
    // alpha, we could use GL_RGBA here. We don't need it though, because we didn't load our image with alpha support so
    // it would be a waste of memory. That second zero is required to be zero. Don't ask. The second GL_RGB specifies
    // the encoding of our texture data. Then we specify that each color channel is encoded in an unsigned byte.
    // stbi_load() dictates these values. Finally, we pass the raw texture data.
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texture_w, texture_h, 0, GL_RGB, GL_UNSIGNED_BYTE, texture_data);

    stbi_image_free(texture_data);

    return tex_id;
}

// Loads a texture, but also loads a fourth alpha channel (for PNG images).
unsigned int load_texture_alpha(const char* file_name) {
    int texture_w, texture_h, texture_channels;
    // This loads a PNG file into memory as an array of RGBA pixels.
    // We specify that we do care about the alpha channel, by asking for 4 channels.
    // This function gives us the texture's size and how many channels it actually loaded.
    unsigned char* texture_data = stbi_load(file_name, &texture_w, &texture_h, &texture_channels, 4);

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
    // Mipmaps are a way to simplify textures when they are very small, in order to more efficiently render far-away
    // objects. If you've ever played a 3D video game, its related to Level of Detail (LOD). We haven't enabled
    // mipmapping (it can be enabled via glTexParameter(), but we don't need it for 2D), so 0 specifies the default
    // mipmap level. The first GL_RGB specifies how we want OpenGL to represent the texture internally. If we needed
    // alpha, we could use GL_RGBA here. That second zero is required to be zero. Don't ask. The second GL_RGBA
    // specifies the encoding of our texture data. Then we specify that each color channel is encoded in an unsigned
    // byte. stbi_load() dictates these values. Finally, we pass the raw texture data.
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, texture_w, texture_h, 0, GL_RGBA, GL_UNSIGNED_BYTE, texture_data);

    stbi_image_free(texture_data);

    return tex_id;
}

void set_stopwatch_icon_color(Session *bs_session) {
    switch (bs_session->stopwatch.state) {
        case StopwatchState::RUNNING:
            glColor4f(0.0f, 1.0f, 0.0f, 1.0f);
            break;
        case StopwatchState::PAUSED:
            glColor4f(1.0f, 0.67f, 0.0f, 1.0f);
            break;
        default:
            glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
            break;
    }
}

const char* get_stopwatch_text(Session *bs_session) {
    static char buffer[50];

    if (bs_session->stopwatch.state == StopwatchState::STOPPED) {
        sprintf(buffer, "00:00:00");

        return buffer;
    }

    unsigned int time_to_use;

    if (bs_session->stopwatch.state == StopwatchState::PAUSED) {
        time_to_use = bs_session->stopwatch.pause_time;
    } else {
        time_to_use = bs_session->global_clock.get_millis();
    }

    unsigned int millis = time_to_use - bs_session->stopwatch.start_time;

    unsigned int seconds = millis / 1000;
    unsigned int minutes = seconds / 60;
    seconds = seconds % 60;
    unsigned int hours = minutes / 60;
    minutes = minutes % 60;

    sprintf(buffer, "%02d:%02d:%02d", hours, minutes, seconds);

    return buffer;
}

void do_info_panel(Layout* layout, int width, int height, Session *bs_session) {
    static char info_buffer[200];

    int x = layout->current_x;
    int y = layout->current_y;

    //int w = 445;
    //int h = 300;

    do_solid_rect(layout, width, height, 68.0f / 255.0f, 68.0f / 255.0f, 68.0f / 255.0f);

    int y_offset = 5;
    sprintf(info_buffer, "Subsystem Computer Net Status: %s", bs_session->r_feed.status == network::FeedStatus::ALIVE ? "alive" : "dead");
    (bs_session->r_feed.status == network::FeedStatus::ALIVE) ? glColor4f(0.0f, 1.0f, 0.0f, 1.0f) : glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
    draw_text(&gui::state.global_font, info_buffer, x + 5, y + y_offset, 14);
    
    y_offset += 15;
    sprintf(info_buffer, "Video Computer Net Status: %s", bs_session->v_feed.status == network::FeedStatus::ALIVE ? "alive" : "dead");
    (bs_session->v_feed.status == network::FeedStatus::ALIVE) ? glColor4f(0.0f, 1.0f, 0.0f, 1.0f) : glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
    draw_text(&gui::state.global_font, info_buffer, x + 5, y + y_offset, 14);

    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);

    y_offset += 25;
    sprintf(info_buffer, "Net Thpt (bs/r/v): %.2f/%.2f/%.2f MiB/s", bs_session->bs_tp, bs_session->r_tp, bs_session->v_tp);
    draw_text(&gui::state.global_font, info_buffer, x + 5, y + y_offset, 14);

    y_offset += 15;
    sprintf(info_buffer, "Total Net Thpt: %.2f MiB/s", bs_session->t_tp);
    draw_text(&gui::state.global_font, info_buffer, x + 5, y + y_offset, 14);

    y_offset += 25;
    switch (waypoint::rover_fix) {
        case network::LocationMessage::FixStatus::NONE:
            glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
            sprintf(info_buffer, "No GPS fix!");
            draw_text(&gui::state.global_font, info_buffer, x + 5, y + y_offset, 14);
            break;
        case network::LocationMessage::FixStatus::STABILIZING:
            glColor4f(1.0f, 0.5f, 0.0f, 1.0f);
            sprintf(info_buffer, "Rover lat/lon: %.6f,%.6f", waypoint::rover_latitude, waypoint::rover_longitude);
            draw_text(&gui::state.global_font, info_buffer, x + 5, y + y_offset, 14);
            break;
        case network::LocationMessage::FixStatus::FIXED:
            glColor4f(1.0f, 1.0f, 1.0f, 1.0f);        
            sprintf(info_buffer, "Rover lat/lon: %.6f,%.6f", waypoint::rover_latitude, waypoint::rover_longitude);
            break;
        default:
            assert(false);
            break;
    }
    draw_text(&gui::state.global_font, info_buffer, x + 5, y + y_offset, 14);

    y_offset += 25;
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    sprintf(info_buffer, "Subsystem CPU TPS: %.0f", bs_session->last_subsystem_tick);
    draw_text(&gui::state.global_font, info_buffer, x + 5, y + y_offset, 14);

    y_offset += 15;
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    sprintf(info_buffer, "Video CPU TPS: %.0f", bs_session->last_video_tick);
    draw_text(&gui::state.global_font, info_buffer, x + 5, y + y_offset, 14);

    y_offset += 25;
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    switch (bs_session->bs_focus_mode){
    case FocusMode::GENERAL:{
        glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
        sprintf(info_buffer, "BS Focus Mode: General");
        draw_text(&gui::state.global_font, info_buffer, x + 5, y + y_offset, 14);
        break;
    }
    case FocusMode::DRIVE:{
        glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
        sprintf(info_buffer, "BS Focus Mode: Drive");
        draw_text(&gui::state.global_font, info_buffer, x + 5, y + y_offset, 14);
        break;
    }
    case FocusMode::ARM:{
        glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
        sprintf(info_buffer, "BS Focus Mode: Arm");
        draw_text(&gui::state.global_font, info_buffer, x + 5, y + y_offset, 14);
        break;
    }
    case FocusMode::SCIENCE:{
        glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
        sprintf(info_buffer, "BS Focus Mode: Science");
        draw_text(&gui::state.global_font, info_buffer, x + 5, y + y_offset, 14);
        break;
    }
    case FocusMode::AUTONOMY:{
        glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
        sprintf(info_buffer, "BS Focus Mode: Autonomy");
        draw_text(&gui::state.global_font, info_buffer, x + 5, y + y_offset, 14);
        break;
    }
    default:
        break;
    }

    y_offset += 15;
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    switch (bs_session->subsystem_focus_mode){
    case network::FocusModeMessage::FocusMode::GENERAL:{
        glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
        sprintf(info_buffer, "Subsystem CPU Focus Mode: General");
        draw_text(&gui::state.global_font, info_buffer, x + 5, y + y_offset, 14);
        break;
    }
    case network::FocusModeMessage::FocusMode::DRIVE:{
        glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
        sprintf(info_buffer, "Subsystem CPU Focus Mode: Drive");
        draw_text(&gui::state.global_font, info_buffer, x + 5, y + y_offset, 14);
        break;
    }
    case network::FocusModeMessage::FocusMode::ARM:{
        glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
        sprintf(info_buffer, "Subsystem CPU Focus Mode: Arm");
        draw_text(&gui::state.global_font, info_buffer, x + 5, y + y_offset, 14);
        break;
    }
    case network::FocusModeMessage::FocusMode::SCIENCE:{
        glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
        sprintf(info_buffer, "Subsystem CPU Focus Mode: Science");
        draw_text(&gui::state.global_font, info_buffer, x + 5, y + y_offset, 14);
        break;
    }
    case network::FocusModeMessage::FocusMode::AUTONOMY:{
        glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
        sprintf(info_buffer, "Subsystem CPU Focus Mode: Autonomy");
        draw_text(&gui::state.global_font, info_buffer, x + 5, y + y_offset, 14);
        break;
    }
    default:
        break;
    }

    y_offset += 15;
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    switch (bs_session->video_focus_mode){
    case network::FocusModeMessage::FocusMode::GENERAL:{
        glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
        sprintf(info_buffer, "Video CPU Focus Mode: General");
        draw_text(&gui::state.global_font, info_buffer, x + 5, y + y_offset, 14);
        break;
    }
    case network::FocusModeMessage::FocusMode::DRIVE:{
        glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
        sprintf(info_buffer, "Video CPU Focus Mode: Drive");
        draw_text(&gui::state.global_font, info_buffer, x + 5, y + y_offset, 14);
        break;
    }
    case network::FocusModeMessage::FocusMode::ARM:{
        glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
        sprintf(info_buffer, "Video CPU Focus Mode: Arm");
        draw_text(&gui::state.global_font, info_buffer, x + 5, y + y_offset, 14);
        break;
    }
    case network::FocusModeMessage::FocusMode::SCIENCE:{
        glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
        sprintf(info_buffer, "Video CPU Focus Mode: Science");
        draw_text(&gui::state.global_font, info_buffer, x + 5, y + y_offset, 14);
        break;
    }
    case network::FocusModeMessage::FocusMode::AUTONOMY:{
        glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
        sprintf(info_buffer, "Video CPU Focus Mode: Autonomy");
        draw_text(&gui::state.global_font, info_buffer, x + 5, y + y_offset, 14);
        break;
    }
    default:
        break;
    }


    y_offset += 25;
    std::unordered_map<std::string, double>::iterator itr = bs_session->power_sub_info.begin();
    while(itr != bs_session->power_sub_info.end()){
        sprintf(info_buffer, "%s: %.3f", itr->first.c_str(), itr->second);
        glColor4f(1.0f, 1.0f, 0.5f, 1.0f);
        draw_text(&gui::state.global_font, info_buffer, x + 5, y + y_offset, 14);
        y_offset += 15;
        itr++;
    }

    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    time_t current_time;
    time(&current_time);
    struct tm* time_info = localtime(&current_time);

    strftime(info_buffer, sizeof(info_buffer), "%I:%M:%S", time_info);

    draw_text(&gui::state.global_font, info_buffer, x + 5, y + height - 20 - 5, 20);

    auto stopwatch_buffer = get_stopwatch_text(bs_session);

    int stopwatch_text_width = text_width(&gui::state.global_font, stopwatch_buffer, 20);

    draw_text(&gui::state.global_font, stopwatch_buffer, x + width - 5 - stopwatch_text_width, y + height - 20 - 5, 20);

    set_stopwatch_icon_color(bs_session);
    fill_textured_rect_mix_color(x + width - 5 - stopwatch_text_width - 3 - 20, y + height - 20 - 5, 20, 20, bs_session->stopwatch_texture_id);
}

void do_subsystem_panel(Layout* layout, int width, int height, Session *bs_session){
    static char text_buffer[200];
    
    int x = layout->current_x;
    int y = layout->current_y;
    
    //int w = 445;
    //int h = 300;
    
    do_solid_rect(layout, width, height, 68.0f / 255.0f, 68.0f / 255.0f, 68.0f / 255.0f);

    //Print contents of subsystem data registers into subsystem window by focus mode
    std::unordered_map<std::string, double>::iterator itr;
    int offset = 0;
    switch(bs_session->bs_focus_mode){
        case FocusMode::DRIVE:{
            sprintf(text_buffer, "Drive Subsystem Info");
            glColor4f(1.0f, 1.0f, 0.5f, 1.0f);
            draw_text(&gui::state.global_font, text_buffer, x + 5, y + offset + 5, 20);
            offset += 30;
            itr = bs_session->drive_sub_info.begin();
            while(itr != bs_session->drive_sub_info.end()){
                sprintf(text_buffer, "%s: %.3f", itr->first.c_str(), itr->second);
                glColor4f(1.0f, 1.0f, 1.5f, 1.0f);
                draw_text(&gui::state.global_font, text_buffer, x + 5, y + offset + 5, 15);
                offset += 20;
                itr++;
            }
            break;
        }
        case FocusMode::ARM:{
            sprintf(text_buffer, "Arm Subsystem Info");
            glColor4f(1.0f, 1.0f, 0.5f, 1.0f);
            draw_text(&gui::state.global_font, text_buffer, x + 5, y + offset + 5, 20);
            offset += 30;
            itr = bs_session->arm_sub_info.begin();
            while(itr != bs_session->arm_sub_info.end()){
                sprintf(text_buffer, "%s: %.3f", itr->first.c_str(), itr->second);
                glColor4f(1.0f, 1.0f, 1.5f, 1.0f);
                draw_text(&gui::state.global_font, text_buffer, x + 5, y + offset + 5, 15);
                offset += 20;
                itr++;
            }
            break;
        }
        case FocusMode::SCIENCE:{
            sprintf(text_buffer, "Science Subsystem Info");
            glColor4f(1.0f, 1.0f, 0.5f, 1.0f);
            draw_text(&gui::state.global_font, text_buffer, x + 5, y + offset + 5, 20);
            offset += 30;
            itr = bs_session->science_sub_info.begin();
            while(itr != bs_session->science_sub_info.end()){
                sprintf(text_buffer, "%s: %.3f", itr->first.c_str(), itr->second);
                glColor4f(1.0f, 1.0f, 1.5f, 1.0f);
                draw_text(&gui::state.global_font, text_buffer, x + 5, y + offset + 5, 15);
                offset += 20;
                itr++;
            }
            break;
        }
        case FocusMode::AUTONOMY:{
            sprintf(text_buffer, "Autonomy Subsystem Info");
            glColor4f(1.0f, 1.0f, 0.5f, 1.0f);
            draw_text(&gui::state.global_font, text_buffer, x + 5, y + offset + 5, 20);
            offset += 30;
            itr = bs_session->autonomy_sub_info.begin();
            while(itr != bs_session->autonomy_sub_info.end()){
                sprintf(text_buffer, "%s: %.3f", itr->first.c_str(), itr->second);
                glColor4f(1.0f, 1.0f, 1.5f, 1.0f);
                draw_text(&gui::state.global_font, text_buffer, x + 5, y + offset + 5, 15);
                offset += 20;
                itr++;
            }
            break;
        }
        case FocusMode::GENERAL:
        default:{

        }
    }
}

void do_stopwatch_menu(Session *bs_session){
    const int w = 150;
    const int h = 110;

    const int x = WINDOW_WIDTH - 20 - w;
    const int y = WINDOW_HEIGHT - 20 - h;

    glColor4f(0.2f, 0.2f, 0.2f, 1.0f);
    fill_rectangle(x, y, w, h);

    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    draw_text(&gui::state.global_font, "Stopwatch", x + 5, y + 5, 20);

    const char* space_help_text;
    switch (bs_session->stopwatch.state) {
        case StopwatchState::STOPPED:
            space_help_text = "<space> : start";
            break;
        case StopwatchState::PAUSED:
            space_help_text = "<space> : resume";
            break;
        case StopwatchState::RUNNING:
            space_help_text = "<space> : pause";
            break;
    }

    draw_text(&gui::state.global_font, space_help_text, x + 5, y + 5 + 20 + 15, 15);
    draw_text(&gui::state.global_font, "r       : reset", x + 5, y + 5 + 20 + 15 + 5 + 15, 15);

    // Draw the actual stopwatch.

    auto stopwatch_buffer = get_stopwatch_text(bs_session);
    int stopwatch_text_width = text_width(&gui::state.global_font, stopwatch_buffer, 20);
    draw_text(&gui::state.global_font, stopwatch_buffer, WINDOW_WIDTH - 20 - stopwatch_text_width - 5, WINDOW_HEIGHT - 20 - 5 - 20, 20);

    set_stopwatch_icon_color(bs_session);
    fill_textured_rect_mix_color(WINDOW_WIDTH - 20 - 5 - stopwatch_text_width - 3 - 20,
        WINDOW_HEIGHT - 20 - 5 - 20, 20, 20, bs_session->stopwatch_texture_id);
}

void do_help_menu(std::vector<const char*> commands, std::vector<const char*> debug_commands, Session *bs_session) {
    // Texts have a fixed height and spacing
    int height = 20;
    int spacing = height + 10;

    const char* title = "Help Menu";
    int title_height = 25;
    int title_width = text_width(&gui::state.global_font, title, title_height);
    int debug_title_height = 25;
    const char* debug_title = "Debug Commands";
    int debug_title_width = text_width(&gui::state.global_font, debug_title, debug_title_height);
    const char* exit_prompt = "Press 'escape' to exit menu";

    int max_width = debug_title_width;
    for (unsigned int i = 0; i < commands.size(); i++) {
        int current_width = text_width(&gui::state.global_font, commands[i], height);
        if (current_width > max_width) max_width = current_width;
    }
    for (unsigned int i = 0; i < debug_commands.size(); i++) {
        int current_width = text_width(&gui::state.global_font, debug_commands[i], height);
        if (current_width > max_width) max_width = current_width;
    }

    int right_padding = 150;
    int bottom_padding = (spacing * 1.5) + 15; // The +15 is to account for the exit prompt
    int top_padding = 20;
    int menu_width = max_width + right_padding;
    int menu_height = (spacing * commands.size()) + (spacing * debug_commands.size()) + title_height +
        debug_title_height + top_padding + bottom_padding;

    Layout help_layout{};

    help_layout.advance_x((WINDOW_WIDTH / 2) - (menu_width / 2));
    help_layout.advance_y((WINDOW_HEIGHT / 2) - (menu_height / 2));
    help_layout.push();

    int x = help_layout.current_x;
    int y = help_layout.current_y;

    do_solid_rect(&help_layout, menu_width, menu_height, 0.2, 0.2f, 0.2);

    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    int left_margin = 5;

    draw_text(&gui::state.global_font, title, x + (menu_width / 2) - (title_width / 2), y, title_height);

    for (unsigned int i = 0; i < commands.size(); i++) {
        draw_text(&gui::state.global_font, commands[i], x + left_margin, y + (spacing * i) + top_padding + title_height, height);
    }

    glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
    int debug_commands_x = x + (menu_width / 2) - (debug_title_width / 2);
    int debug_commands_y = y + (spacing * commands.size()) + top_padding + title_height;
    draw_text(&gui::state.global_font, debug_title, debug_commands_x, debug_commands_y, debug_title_height);
    debug_commands_y += 10; // To give extra room for the commands after the debug title

    for (unsigned int i = 0; i < debug_commands.size(); i++) {
        draw_text(&gui::state.global_font, debug_commands[i], x + left_margin, debug_commands_y + (spacing * (i + 1)), height);
    }

    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    int exit_prompt_width = text_width(&gui::state.global_font, exit_prompt, 15);
    draw_text(&gui::state.global_font, exit_prompt, x + (menu_width / 2) - (exit_prompt_width / 2), y + menu_height - 20, 15);

    glColor4f(0.0f, 0.0f, 0.0f, 1.0f);
    glLineWidth(4.0f);
    glBegin(GL_LINES);

    // Footer
    glVertex2f(x, y + menu_height - 20);
    glVertex2f(x + menu_width, y + menu_height - 20);

    // Title Line
    glVertex2f(x, y + title_height + 10);
    glVertex2f(x + menu_width, y + title_height + 10);

    // Before debug title
    glVertex2f(x, debug_commands_y - 8);
    glVertex2f(x + menu_width, debug_commands_y - 8);

    // After debug title
    glVertex2f(x, debug_commands_y + debug_title_height + 2);
    glVertex2f(x + menu_width, debug_commands_y + debug_title_height + 2);

    glEnd();

    help_layout.pop();
}

void do_lidar(Layout* layout, Session *bs_session) {
    int wx = layout->current_x;
    int wy = layout->current_y;

    do_solid_rect(layout, 300, 300, 0, 0, 0);

    glColor4f(0.0f, 0.0f, 1.0f, 1.0f);

    glBegin(GL_QUADS);

    for (size_t i = 0; i < bs_session->lidar_points.size(); i++) {
        float angle = 225.0f - (float) i;
        float theta = angle * M_PI / 180.0f;

        theta -= M_PI;

        uint16_t dist = bs_session->lidar_points.at(i);

        float r = dist * 150.0f / 10000.0f;

        float hs = 1.0f;

        float x = wx + 150.0f + r * cosf(theta);
        float y = wy + 150.0f + r * sinf(theta);

        if (dist < 100) continue;

        glVertex2f(x - hs, y - hs);
        glVertex2f(x + hs, y - hs);
        glVertex2f(x + hs, y + hs);
        glVertex2f(x - hs, y + hs);
    }

    float hs = 1.2 * 15.0f / 2.0f;

    float x = wx + 150.0f;
    float y = wy + 150.0f;

    glColor4f(1.0f, 0.0f, 0.0f, 1.0f);

    glVertex2f(x - hs, y - hs);
    glVertex2f(x + hs, y - hs);
    glVertex2f(x + hs, y + hs);
    glVertex2f(x - hs, y + hs);

    glEnd();
}

void do_camera_move_target(Session *bs_session) {
    const float COVER_ALPHA = 0.5f;
    glColor4f(1.0f, 1.0f, 1.0f, COVER_ALPHA);
    fill_rectangle(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);

    const int PRIMARY_TEXT_SIZE = 50;
    const int SECONDARY_TEXT_SIZE = 30;

    const char* primary_text = "Press '0' to move feed here";
    const char* secondary_text = "Press '1' to move feed here";

    // TODO: These are hardcoded from the do_gui layout stuff.
    // Maybe calculate the positions of everything at runtime start?
    const int X = 20 + 572 + 10;

    const int Y_PRIMARY = 20;
    const int Y_SECONDARY = Y_PRIMARY + PRIMARY_FEED_HEIGHT + 10;

    int primary_text_width = text_width(&gui::state.global_font, primary_text, PRIMARY_TEXT_SIZE);
    int secondary_text_width = text_width(&gui::state.global_font, secondary_text, SECONDARY_TEXT_SIZE);

    int ptx = X + (PRIMARY_FEED_WIDTH / 2) - (primary_text_width / 2);
    int pty = Y_PRIMARY + (PRIMARY_FEED_HEIGHT / 2);

    int stx = X + (SECONDARY_FEED_WIDTH / 2) - (secondary_text_width / 2);
    int sty = Y_SECONDARY + (SECONDARY_FEED_HEIGHT / 2);

    const int BG_PADDING = 10;

    glColor4f(0.0f, 0.0f, 1.0f, 1.0f);
    fill_rectangle(ptx - BG_PADDING, pty - BG_PADDING, primary_text_width + 2 * BG_PADDING, PRIMARY_TEXT_SIZE + 2 * BG_PADDING);
    fill_rectangle(stx - BG_PADDING, sty - BG_PADDING, secondary_text_width + 2 * BG_PADDING, SECONDARY_TEXT_SIZE + 2 * BG_PADDING);

    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    draw_text(&gui::state.global_font, primary_text, ptx, pty, PRIMARY_TEXT_SIZE);
    draw_text(&gui::state.global_font, secondary_text, stx, sty, SECONDARY_TEXT_SIZE);
}

void do_camera_matrix(Session *bs_session) {
    switch (state.input_state) {
        case InputState::CAMERA_MATRIX:
        case InputState::CAMERA_MOVE:
            break;
        case InputState::CAMERA_MOVE_TARGET:
            do_camera_move_target(bs_session);
            return;
        default:
            return;
    }

    const int SIDE_MARGIN = 50;
    const int TOP_MARGIN = 50;

    const int BACKGROUND_SHADE = 40;

    const int MATRIX_WIDTH = WINDOW_WIDTH - 2 * SIDE_MARGIN;
    const int MATRIX_HEIGHT = WINDOW_HEIGHT - 2 * SIDE_MARGIN;

    const int MATRIX_PADDING = 25;

    glColor4f(BACKGROUND_SHADE / 255.0f, BACKGROUND_SHADE / 255.0f, BACKGROUND_SHADE / 255.0f, 1.0f);
    fill_rectangle(SIDE_MARGIN, TOP_MARGIN, MATRIX_WIDTH, MATRIX_HEIGHT);

    const int VIEW_WIDTH = 500;
    const int VIEW_HEIGHT = VIEW_WIDTH * 9 / 16;
    const int NUM_VIEWS_PER_ROW = (MATRIX_WIDTH + MATRIX_PADDING) / (VIEW_WIDTH + MATRIX_PADDING);
    const int NUM_ROWS = (MAX_FEEDS + NUM_VIEWS_PER_ROW - 1) / NUM_VIEWS_PER_ROW;
    const int WIDTH_OF_ROW = NUM_VIEWS_PER_ROW * (VIEW_WIDTH + MATRIX_PADDING) + MATRIX_PADDING;
    const int EXTRA_SIDE_PADDING = (MATRIX_WIDTH - WIDTH_OF_ROW) / 2;

    const int TEXT_PADDING = 5;
    const int TEXT_SIZE = 20;

    // Stuff will overflow if we go over one digit!
    // Plus 2 for ": ".
    const int TEXT_MAX_LEN = camera_feed::FEED_NAME_MAX_LEN + 1 + 2;
    // Can be equal since ids are 0-9.
    assert(MAX_FEEDS <= 10);

    int feed_idx = 0;
    for (int r = 0; r < NUM_ROWS; r++) {
        for (int c = 0; c < NUM_VIEWS_PER_ROW; c++) {
            if (feed_idx >= MAX_FEEDS) break;

            int view_x = SIDE_MARGIN + MATRIX_PADDING + EXTRA_SIDE_PADDING + c * (VIEW_WIDTH + MATRIX_PADDING);
            int view_y = TOP_MARGIN + MATRIX_PADDING + r * (VIEW_HEIGHT + MATRIX_PADDING);

            fill_textured_rect(view_x, view_y, VIEW_WIDTH, VIEW_HEIGHT, bs_session->camera_feeds[feed_idx].gl_texture_id);

            glColor4f(1.0f, 1.0f, 1.0f, 1.0f);

            static char feed_display_buffer[TEXT_MAX_LEN + 1];
            snprintf(feed_display_buffer, sizeof(feed_display_buffer), "%d: %s", feed_idx, bs_session->camera_feeds[feed_idx].name);

            draw_text(&gui::state.global_font, feed_display_buffer, view_x + TEXT_PADDING, view_y + VIEW_HEIGHT - TEXT_SIZE - TEXT_PADDING,TEXT_SIZE);

            feed_idx++;
        }
    }

    const int HELP_TEXT_SIZE = 15;

    const char* help_text;
    if (state.input_state == InputState::CAMERA_MATRIX) {
        help_text = "m: move camera | escape: exit matrix";
    } else if (state.input_state == InputState::CAMERA_MOVE) {
        help_text = "escape: cancel move";
    } else {
        help_text = "?";
    }

    int help_text_width = text_width(&gui::state.global_font, help_text, HELP_TEXT_SIZE);

    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    draw_text(&gui::state.global_font, help_text, WINDOW_WIDTH - SIDE_MARGIN - TEXT_PADDING - help_text_width, 
        WINDOW_HEIGHT - TOP_MARGIN - TEXT_PADDING - HELP_TEXT_SIZE, HELP_TEXT_SIZE);

    if (state.input_state == InputState::CAMERA_MOVE) {
        glColor4f(255.0f / 255.0f, 199.0f / 255.0f, 0.0f, 1.0f);
        draw_text(&gui::state.global_font, "Press key '0'-'9' to select feed to move",
            SIDE_MARGIN + TEXT_PADDING, WINDOW_HEIGHT - TOP_MARGIN - TEXT_PADDING - HELP_TEXT_SIZE, HELP_TEXT_SIZE);
    }
}

void do_autonomy_control(Session *bs_session) {
    if (gui::state.input_state != gui::InputState::AUTONOMY_CONTROL
        && gui::state.input_state != gui::InputState::AUTONOMY_EDIT_TARGET) return;

    const int WIDTH = 600;
    const int HEIGHT = 800;
    const int PADDING = 10;
    const int BACKGROUND_SHADE = 40;

    const int PANEL_X = WINDOW_WIDTH / 2 - WIDTH / 2;
    const int PANEL_Y = WINDOW_HEIGHT / 2 - HEIGHT / 2;

    int x = PANEL_X + PADDING;
    int y = PANEL_Y + PADDING;

    glColor4f(BACKGROUND_SHADE / 255.0f, BACKGROUND_SHADE / 255.0f, BACKGROUND_SHADE / 255.0f, 1.0f);
    gui::fill_rectangle(PANEL_X, PANEL_Y, WIDTH, HEIGHT);

    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    gui::draw_text(&gui::state.global_font, "Autonomy Control Panel", x, y, 30);
    
    y += 30 + 20;

    char text_buffer[500];

    const char* status_text;
    switch (bs_session->autonomy_info.status) {
        case network::AutonomyStatusMessage::Status::IDLE:
            status_text = "idle";
            break;
        case network::AutonomyStatusMessage::Status::NAVIGATING:
            status_text = "navigating";
            break;
        case network::AutonomyStatusMessage::Status::DONE:
            status_text = "done";
            break;
    }

    sprintf(text_buffer, "Status: %s", status_text);
    gui::draw_text(&gui::state.global_font, text_buffer, x, y, 20);

    y += 20 + 10;

    if (gui::state.input_state == gui::InputState::AUTONOMY_EDIT_TARGET) {
        int target_width = text_width(&gui::state.global_font, "Target:", 20);
        int lat_width = text_width(&gui::state.global_font, bs_session->autonomy_info.edit_lat.c_str(), 20);
        int comma_width = text_width(&gui::state.global_font, ",", 20);
        int lon_width = text_width(&gui::state.global_font, bs_session->autonomy_info.edit_lon.c_str(), 20);

        gui::draw_text(&gui::state.global_font, "Target:", x, y, 20);
        
        if (bs_session->autonomy_info.edit_idx == 0) {
            glColor4f(0.0f, 0.0f, 0.0f, 1.0f);
            gui::fill_rectangle(x + target_width + 10, y, lat_width, 20);
        }

        glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
        gui::draw_text(&gui::state.global_font, bs_session->autonomy_info.edit_lat.c_str(), x + target_width + 10, y, 20);

        gui::draw_text(&gui::state.global_font, ",", x + target_width + 10 + lat_width + 10, y, 20);

        if (bs_session->autonomy_info.edit_idx == 1) {
            glColor4f(0.0f, 0.0f, 0.0f, 1.0f);
            gui::fill_rectangle(x + target_width + 10 + lat_width + 10 + comma_width + 10, y, lon_width, 20);
        }

        glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
        gui::draw_text(&gui::state.global_font, bs_session->autonomy_info.edit_lon.c_str(), x + target_width + 10 + lat_width + 10 + comma_width + 10, y, 20);
    } else {
        if (bs_session->autonomy_info.has_target) {
            sprintf(text_buffer, "Target: %.4f, %.4f", bs_session->autonomy_info.target_lat, bs_session->autonomy_info.target_lon);
        } else {
            sprintf(text_buffer, "No target");
        }
        gui::draw_text(&gui::state.global_font, text_buffer, x, y, 20);
    }

    y += 20 + 10;
}

void do_autonomy_gui(Session *bs_session){
    // Clear the screen to a modern dark gray.
    glClearColor(35.0f / 255.0f, 35.0f / 255.0f, 35.0f / 255.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    Layout layout{};

    // Set margin.
    layout.advance_x(50);
    layout.advance_y(20);
    layout.push();

    // Draw the main camera feed.
    do_textured_rect(&layout, PRIMARY_FEED_WIDTH, PRIMARY_FEED_HEIGHT, bs_session->camera_feeds[bs_session->primary_feed].gl_texture_id);
    
    //Reset X and advance Y to set up for Log Window
    layout.reset_x();
    layout.advance_x(-40);
    layout.advance_y(10);
    layout.push();
    
    //Draw Log Window
    log_view::do_log(&layout, 590, 310, &gui::state.global_font);

    //Reset X and Y to set up for the status window
    layout.reset_y();
    layout.advance_x(10);
    //layout.push();

    //Draw status window
    do_subsystem_panel(&layout, 370, 310, bs_session);

    layout.reset_y();
    layout.advance_x(10);
    
    //Draw info panel window
    do_info_panel(&layout, 380, 310, bs_session);
    
    //Set X and Y for the Waypoint Map Window
    layout.advance_y(-1040);
    layout.advance_x(10);
    layout.push();

    //Draw Waypoint Map Window 
    waypoint_map::do_waypoint_map(&layout, 510, 510);

    //Set X and Y for the LIDAR Window
    layout.reset_x();
    layout.advance_y(20);
    layout.push();

    //Draw the LIDAR Window
    do_lidar(&layout, bs_session);

    int help_text_width = text_width(&gui::state.global_font, "Press 'h' for help", 15);
    glColor4f(0.0f, 0.5f, 0.0f, 1.0f);
    draw_text(&gui::state.global_font, "Press 'h' for help", WINDOW_WIDTH - help_text_width - 2, WINDOW_HEIGHT - 18, 15);

    // Draw the debug overlay.
    layout = {};
    debug_console::do_debug(&layout, &gui::state.global_font);

    // Draw the camera matrix.
    // Note: this currently needs to be called last here in order for the camera movement
    // effects to work properly!
    do_camera_matrix(bs_session);

    do_autonomy_control(bs_session);
}

void do_general_gui(Session *bs_session){
    // Clear the screen to a modern dark gray.
    glClearColor(35.0f / 255.0f, 35.0f / 255.0f, 35.0f / 255.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    Layout layout{};

    // Set margin.
    layout.advance_x(20);
    layout.advance_y(20);
    layout.push();

    // Renders a map that shows where the rover is relative to other waypoints, the current waypoints, and its orientation
    //waypoint_map::do_waypoint_map(&layout,572,572);
    waypoint_map::do_waypoint_map(&layout,590,590);
    
    layout.reset_x();
    layout.advance_y(10);

    // Draw the log.
    log_view::do_log(&layout, LOG_VIEW_WIDTH, LOG_VIEW_HEIGHT, &gui::state.global_font);

    layout.reset_y();
    layout.advance_x(10);
    layout.push();

    // Draw the main camera feed.
    do_textured_rect(&layout, PRIMARY_FEED_WIDTH, PRIMARY_FEED_HEIGHT, bs_session->camera_feeds[bs_session->primary_feed].gl_texture_id);

    layout.reset_x();
    layout.advance_y(10);
    layout.push();

    // Draw the other camera feed.
    layout.reset_y();
    do_textured_rect(&layout, SECONDARY_FEED_WIDTH, SECONDARY_FEED_HEIGHT, bs_session->camera_feeds[bs_session->secondary_feed].gl_texture_id);

    layout.reset_y();
    layout.advance_x(10);

    // Draw the lidar.
    do_lidar(&layout, bs_session);

    layout.reset_y();
    layout.advance_x(10);

    // Draw the info panel.
    do_info_panel(&layout, 400, 310, bs_session);

    int help_text_width = text_width(&gui::state.global_font, "Press 'h' for help", 15);
    glColor4f(0.0f, 0.5f, 0.0f, 1.0f);
    draw_text(&gui::state.global_font, "Press 'h' for help", WINDOW_WIDTH - help_text_width - 2, WINDOW_HEIGHT - 18, 15);

    // Draw the debug overlay.
    layout = {};
    debug_console::do_debug(&layout, &gui::state.global_font);

    // Draw the camera matrix.
    // Note: this currently needs to be called last here in order for the camera movement
    // effects to work properly!
    do_camera_matrix(bs_session);

    do_autonomy_control(bs_session);    
}

void do_drive_gui(Session *bs_session){
    // Clear the screen to a modern dark gray.
    glClearColor(35.0f / 255.0f, 35.0f / 255.0f, 35.0f / 255.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    Layout layout{};
    
    // Set margin.
    layout.advance_x(20);
    layout.advance_y(20);
    layout.push();

    // Renders a map that shows where the rover is relative to other waypoints, the current waypoints, and its orientation
    waypoint_map::do_waypoint_map(&layout,590,590);
    
    layout.reset_x();
    layout.advance_y(10);

    // Draw the log.
    log_view::do_log(&layout, LOG_VIEW_WIDTH, LOG_VIEW_HEIGHT, &gui::state.global_font);

    layout.reset_y();
    layout.advance_x(10);
    layout.push();

    // Draw the main camera feed.
    do_textured_rect(&layout, PRIMARY_FEED_WIDTH, PRIMARY_FEED_HEIGHT, bs_session->camera_feeds[bs_session->primary_feed].gl_texture_id);

    layout.reset_x();
    layout.advance_y(10);
    layout.push();

    //layout.reset_y();
    layout.advance_x(200);

    //// Draw the lidar.
    //do_lidar(&layout, bs_session);

    layout.reset_y();
    layout.advance_x(150);

    //Drive Subsystem Info Window
    do_subsystem_panel(&layout, 520, 310, bs_session);

    layout.reset_y();
    layout.advance_x(10);

    // Draw the info panel.
    do_info_panel(&layout, 400, 310, bs_session);
    
    int help_text_width = text_width(&gui::state.global_font, "Press 'h' for help", 15);
    glColor4f(0.0f, 0.5f, 0.0f, 1.0f);
    draw_text(&gui::state.global_font, "Press 'h' for help", WINDOW_WIDTH - help_text_width - 2, WINDOW_HEIGHT - 18, 15);

    // Draw the debug overlay.
    layout = {};
    debug_console::do_debug(&layout, &gui::state.global_font);

    // Draw the camera matrix.
    // Note: this currently needs to be called last here in order for the camera movement
    // effects to work properly!
    do_camera_matrix(bs_session);

    //do_autonomy_control(bs_session);
}

void do_arm_gui(Session *bs_session){
    // Clear the screen to a modern dark gray.
    glClearColor(35.0f / 255.0f, 35.0f / 255.0f, 35.0f / 255.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    Layout layout{};

    // Set margin.
    layout.advance_x(20);
    layout.advance_y(20);
    layout.push();

    // PLACEHOLDER FOR SUBSYSTEM INFO WINDOW
    //do_solid_rect(&layout, 590, 510, 0, 192, 171);
    do_subsystem_panel(&layout, 590, 520, bs_session);

    //Resets x and sets a margin of 10px inbetween subsystem and log windows
    layout.reset_x();
    layout.advance_y(10);

    //Sets log window to 590x510
    log_view::do_log(&layout, 590, 510, &gui::state.global_font);

    //Starts drawing from the top of the screen 10px to the right of the end of the log window
    layout.reset_y();
    layout.advance_x(10);
    layout.push();

    // Draw the main camera feed at 1280x720
    do_textured_rect(&layout, 1280, 720, bs_session->camera_feeds[bs_session->primary_feed].gl_texture_id);

    //Setting drawing point to below main camera
    layout.reset_x();
    layout.advance_y(10);
    layout.push();
    
    //accounting for empty space
    layout.advance_x(560);
    layout.push();

    // Renders the waypoint map at 310x310
    waypoint_map::do_waypoint_map(&layout,310,310);

    //Places space between waypoint map and status window
    layout.reset_y();
    layout.advance_x(10);
    layout.push();

    int help_text_width = text_width(&gui::state.global_font, "Press 'h' for help", 15);
    glColor4f(0.0f, 0.5f, 0.0f, 1.0f);
    draw_text(&gui::state.global_font, "Press 'h' for help", WINDOW_WIDTH - help_text_width - 2, WINDOW_HEIGHT - 18, 15);

    // Draw the info panel.
    do_info_panel(&layout, 400, 310, bs_session);

    //Necessary for proper camera usage
    do_camera_matrix(bs_session);
}

void do_science_gui(Session *bs_session){
    // Clear the screen to a modern dark gray.
    glClearColor(35.0f / 255.0f, 35.0f / 255.0f, 35.0f / 255.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    Layout layout{};

    // Set margin.
    layout.advance_x(20);
    layout.advance_y(20);
    layout.push();

    do_subsystem_panel(&layout, 590, 520, bs_session);

    //Resets x and sets a margin of 10px inbetween subsystem and log windows
    
    layout.reset_x();
    layout.advance_y(10);

    //Draw log window
    log_view::do_log(&layout, 590, 510, &gui::state.global_font);

    layout.reset_y();
    layout.advance_x(10);
    layout.push();

    // Draw the main camera feed
    do_textured_rect(&layout, 1280, 720, bs_session->camera_feeds[bs_session->primary_feed].gl_texture_id);

    layout.reset_x();
    layout.advance_y(10);
    layout.push();

    // Draw the secondary camera feed
    do_textured_rect(&layout, SECONDARY_FEED_WIDTH, SECONDARY_FEED_HEIGHT, bs_session->camera_feeds[bs_session->secondary_feed].gl_texture_id);

    //layout.reset_y();
    layout.reset_y();
    layout.advance_x(330);

    // Draw info panel window
    do_info_panel(&layout, 400, 310, bs_session);

    int help_text_width = text_width(&gui::state.global_font, "Press 'h' for help", 15);
    glColor4f(0.0f, 0.5f, 0.0f, 1.0f);
    draw_text(&gui::state.global_font, "Press 'h' for help", WINDOW_WIDTH - help_text_width - 2, WINDOW_HEIGHT - 18, 15);

    //Necessary for proper camera usage
    do_camera_matrix(bs_session);
}

void do_gui(Session *bs_session) {
    switch (bs_session->bs_focus_mode)
    {
    case FocusMode::GENERAL:{
        do_general_gui(bs_session);
        break;
    }
    case FocusMode::ARM:{
        do_arm_gui(bs_session);
        break;
    }
    case FocusMode::DRIVE:{
        do_drive_gui(bs_session);
        break;
    }
    case FocusMode::SCIENCE:{
        do_science_gui(bs_session);
        break;
    } 
    case FocusMode::AUTONOMY:{
        do_autonomy_gui(bs_session);
        break;
    }
    default:
        break;
    }
}

void glfw_character_callback(GLFWwindow* window, unsigned int codepoint) {
    if (gui::state.input_state == gui::InputState::DEBUG_CONSOLE) {
        if (codepoint < 128) {
            gui::debug_console::handle_input((char) codepoint);
        }
    }
}

void glfw_key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    void *bs_session_pointer = glfwGetWindowUserPointer(window);
    Session *bs_session = static_cast<Session *>(bs_session_pointer);
    bool z_on = glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS;
    if (gui::state.input_state == gui::InputState::DEBUG_CONSOLE) {
        if (action == GLFW_PRESS || action == GLFW_REPEAT) {
            gui::debug_console::handle_keypress(key, mods, bs_session);
        }
    } 
    else if (gui::state.input_state == gui::InputState::KEY_COMMAND) {
        // We open the menu on release to prevent the D key from being detected
        // in the character callback upon release.
        if (action == GLFW_RELEASE && key == GLFW_KEY_D) {
            gui::state.show_debug_console = true;
            gui::state.input_state = gui::InputState::DEBUG_CONSOLE;
        } else if (action == GLFW_PRESS && key == GLFW_KEY_C) {
            if (mods & GLFW_MOD_SHIFT) {
                gui::state.input_state = gui::InputState::CAMERA_MATRIX;
                bs_session->send_all_feeds();
            } else {
                int temp = bs_session->primary_feed;
                bs_session->primary_feed = bs_session->secondary_feed;
                bs_session->secondary_feed = temp;
            }
        } 
        else if (z_on && action == GLFW_RELEASE && key == GLFW_KEY_G){
            gui::waypoint_map::gridMap = !gui::waypoint_map::gridMap;
        } 
        else if (action == GLFW_PRESS && key == GLFW_KEY_A) {
            gui::state.input_state = gui::InputState::AUTONOMY_CONTROL;
        } 
        else if (action == GLFW_PRESS && key == GLFW_KEY_M) {
            switch (bs_session->controller_mode) {
                case ControllerMode::DRIVE:
                    bs_session->controller_mode = ControllerMode::ARM;
                    break;
                case ControllerMode::ARM:
                    bs_session->controller_mode = ControllerMode::DRIVE;
                    break;
            }
        } 
        else if (action == GLFW_PRESS && key == GLFW_KEY_1) {
            if(mods & GLFW_MOD_SHIFT) {
                bs_session->bs_focus_mode = FocusMode::GENERAL;
                network::FocusModeMessage message;
                message.focus_mode = network::FocusModeMessage::FocusMode::GENERAL;
                network::publish(&bs_session->bs_feed, &message);
            }
        } 
        else if (action == GLFW_PRESS && key == GLFW_KEY_2) {
            if(mods & GLFW_MOD_SHIFT) {
                bs_session->bs_focus_mode = FocusMode::DRIVE;
                network::FocusModeMessage message;
                message.focus_mode = network::FocusModeMessage::FocusMode::DRIVE;
                network::publish(&bs_session->bs_feed, &message);
            }
        } 
        else if (action == GLFW_PRESS && key == GLFW_KEY_3) {
            if(mods & GLFW_MOD_SHIFT) {
                bs_session->bs_focus_mode = FocusMode::ARM;
                network::FocusModeMessage message;
                message.focus_mode = network::FocusModeMessage::FocusMode::ARM;
                network::publish(&bs_session->bs_feed, &message);
            }
        } else if (action == GLFW_PRESS && key == GLFW_KEY_4) {
            if(mods & GLFW_MOD_SHIFT) {
                bs_session->bs_focus_mode = FocusMode::SCIENCE;
                network::FocusModeMessage message;
                message.focus_mode = network::FocusModeMessage::FocusMode::SCIENCE;
                network::publish(&bs_session->bs_feed, &message);
            }
        } else if (action == GLFW_PRESS && key == GLFW_KEY_5) {
            if(mods & GLFW_MOD_SHIFT) {
                bs_session->bs_focus_mode = FocusMode::AUTONOMY;
                network::FocusModeMessage message;
                message.focus_mode = network::FocusModeMessage::FocusMode::AUTONOMY;
                network::publish(&bs_session->bs_feed, &message);
            }
        }
    } else if (gui::state.input_state == gui::InputState::CAMERA_MATRIX) {
        if (action == GLFW_PRESS && key == GLFW_KEY_ESCAPE) {
            gui::state.input_state = gui::InputState::KEY_COMMAND;
            bs_session->dont_send_invalid();
        } else if (action == GLFW_PRESS && key == GLFW_KEY_M) {
            gui::state.input_state = gui::InputState::CAMERA_MOVE;
        }
    } else if (gui::state.input_state == gui::InputState::CAMERA_MOVE) {
        if (action == GLFW_PRESS && key == GLFW_KEY_ESCAPE) {
            gui::state.input_state = gui::InputState::CAMERA_MATRIX;
        } else if (action == GLFW_PRESS) {
            int selected_feed_idx = -1;
            switch (key) {
                case GLFW_KEY_0:
                    selected_feed_idx = 0;
                    break;
                case GLFW_KEY_1:
                    selected_feed_idx = 1;
                    break;
                case GLFW_KEY_2:
                    selected_feed_idx = 2;
                    break;
                case GLFW_KEY_3:
                    selected_feed_idx = 3;
                    break;
                case GLFW_KEY_4:
                    selected_feed_idx = 4;
                    break;
                case GLFW_KEY_5:
                    selected_feed_idx = 5;
                    break;
                case GLFW_KEY_6:
                    selected_feed_idx = 6;
                    break;
                case GLFW_KEY_7:
                    selected_feed_idx = 7;
                    break;
                case GLFW_KEY_8:
                    selected_feed_idx = 8;
                    break;
                case GLFW_KEY_9:
                    selected_feed_idx = 9;
                    break;
            }

            if (selected_feed_idx >= 0 && selected_feed_idx < MAX_FEEDS) {
                bs_session->feed_to_move = selected_feed_idx;
                gui::state.input_state = gui::InputState::CAMERA_MOVE_TARGET;
            }
        }
    } else if (gui::state.input_state == gui::InputState::CAMERA_MOVE_TARGET) {
        if (action == GLFW_PRESS && key == GLFW_KEY_ESCAPE) {
            gui::state.input_state = gui::InputState::CAMERA_MATRIX;
        } else if (action == GLFW_PRESS && key == GLFW_KEY_0) {
            // If the new feed wasn't being displayed,
            // lets start displaying it
            if (bs_session->feed_to_move != bs_session->secondary_feed) {
                bs_session->send_feed(bs_session->feed_to_move);
            }

            bs_session->primary_feed = bs_session->feed_to_move;
            bs_session->dont_send_invalid();
            gui::state.input_state = gui::InputState::KEY_COMMAND;
        } else if (action == GLFW_PRESS && key == GLFW_KEY_1) {
            // If the new feed isn't the same as it was,
            // lets start displaying it
            if (bs_session->feed_to_move != bs_session->primary_feed) {
                bs_session->send_feed(bs_session->feed_to_move);
            }

            bs_session->secondary_feed = bs_session->feed_to_move;
            bs_session->dont_send_invalid();
            gui::state.input_state = gui::InputState::KEY_COMMAND;
        }
    } else if (gui::state.input_state == gui::InputState::AUTONOMY_CONTROL) {
        if (action == GLFW_PRESS && key == GLFW_KEY_ESCAPE) {
            gui::state.input_state = gui::InputState::KEY_COMMAND;
        } else if (action == GLFW_PRESS && key == GLFW_KEY_T) {
            gui::state.input_state = gui::InputState::AUTONOMY_EDIT_TARGET;
        }
    } else if (gui::state.input_state == gui::InputState::AUTONOMY_EDIT_TARGET) {
        if (action == GLFW_PRESS && key == GLFW_KEY_ESCAPE) {
            gui::state.input_state = gui::InputState::AUTONOMY_CONTROL;
        } else if (action == GLFW_PRESS && key == GLFW_KEY_TAB) {
            bs_session->autonomy_info.edit_idx = (bs_session->autonomy_info.edit_idx + 1) % 2;
        } else if (action == GLFW_PRESS && key == GLFW_KEY_ENTER) {
            // Set has_target to true, save the target, and send the command.
        } else if (action == GLFW_PRESS) {
            std::string& edit_str = bs_session->autonomy_info.edit_idx == 0 ?
                bs_session->autonomy_info.edit_lat : bs_session->autonomy_info.edit_lon;

            switch (key) {
                case GLFW_KEY_0:
                    edit_str.push_back('0');
                    break;
                case GLFW_KEY_1:
                    edit_str.push_back('1');
                    break;
                case GLFW_KEY_2:
                    edit_str.push_back('2');
                    break;
                case GLFW_KEY_3:
                    edit_str.push_back('3');
                    break;
                case GLFW_KEY_4:
                    edit_str.push_back('4');
                    break;
                case GLFW_KEY_5:
                    edit_str.push_back('5');
                    break;
                case GLFW_KEY_6:
                    edit_str.push_back('6');
                    break;
                case GLFW_KEY_7:
                    edit_str.push_back('7');
                    break;
                case GLFW_KEY_8:
                    edit_str.push_back('8');
                    break;
                case GLFW_KEY_9:
                    edit_str.push_back('9');
                    break;
                case GLFW_KEY_MINUS:
                    edit_str.push_back('-');
                    break;
                case GLFW_KEY_PERIOD:
                    edit_str.push_back('.');
                    break;
                case GLFW_KEY_BACKSPACE:
                    if (!edit_str.empty()) edit_str.pop_back();
                    break;
            }
        }
    }
}


} // namespace gui
