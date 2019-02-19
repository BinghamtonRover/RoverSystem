#include "../network/network.hpp"
#include "../shared.hpp"

#include "gui.hpp"
#include "debug_console.hpp"
#include "camera_feed.hpp"
#include "controller.hpp"

#include <GLFW/glfw3.h>
#include <GL/gl.h>

#include <cstdio>
#include <cstdlib>
#include <cstdint>

#include <stack>
#include <string>
#include <vector>
#include <iostream>
#include <chrono>

#include <iostream>
#include <vector>
#include <string>

// Default angular resolution (vertices / radian) to use when drawing circles.
constexpr float ANGULAR_RES = 10.0f;

// Send movement updates 3x per second.
const int MOVMENT_SEND_INTERVAL = 1000/3;

// List of messages the log displays
std::vector<std::string> logMessages;

// Stuff for the box of the log
const int LOG_X = 20;
const int LOG_Y = 600;
const int LOG_WIDTH = 400;
const int LOG_HEIGHT = 400;
const int LOG_THICKNESS = 2;
const unsigned int MAX_CHARS_IN_A_LINE = 32;
const unsigned int MAX_LINES = 15;

// Four parallel lists that are also parallel to logMessages
std::vector<float> red, green, blue, alpha;

// Save the start time so we can use get_ticks.
std::chrono::high_resolution_clock::time_point start_time;

unsigned int get_ticks() {
	auto now = std::chrono::high_resolution_clock::now();

	return (unsigned int) std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
}

void do_gui(camera_feed::Feed feed[4], gui::Font* font) {
    // Clear the screen to a modern dark gray.
    glClearColor(35.0f / 255.0f, 35.0f / 255.0f, 35.0f / 255.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    gui::Layout layout{};

    // Set margin.
    layout.advance_x(20);
    layout.advance_y(20);
    layout.push();

    // Draw the map.
    gui::do_solid_rect(&layout, 572, 572, 119.0f / 255.0f, 82.0f / 255.0f, 65.0f / 255.0f);

    layout.reset_x();
    layout.advance_y(10);

     // Draw the log.
    gui::do_solid_rect(&layout, 572, 458, 0, 0, 0);
    
    layout.reset_y();
    layout.advance_x(10);
    layout.push();

    // Draw the main camera feed.
    gui::do_textured_rect(&layout, 1298, 730, feed[0].gl_texture_id);

    layout.reset_x();
    layout.advance_y(10);
    layout.push();

    // Draw the other camera feed.
	layout.reset_y();
	gui::do_textured_rect(&layout, 533, 300, feed[1].gl_texture_id);

	layout.reset_y();
	layout.advance_x(10);

	gui::do_solid_rect(&layout, 755, 300, 68.0f / 255.0f, 68.0f / 255.0f, 68.0f / 255.0f);

	// Draw the debug overlay.
	layout = {};
	gui::debug_console::do_debug(&layout, font);
}

void glfw_character_callback(GLFWwindow* window, unsigned int codepoint) {
	if (gui::state.input_state == gui::InputState::DEBUG_CONSOLE) {
		if (codepoint < 128) {
			gui::debug_console::handle_input((char) codepoint);
		}
	}
}

void glfw_key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
	if (gui::state.input_state == gui::InputState::DEBUG_CONSOLE) {
		if (action == GLFW_PRESS || action == GLFW_REPEAT) {
			gui::debug_console::handle_keypress(key, mods);
		}
	}
}

// Represents a font.
struct Font {
    // Information that keeps track of each character that we want to be able to draw.
    stbtt_bakedchar baked_chars[95];

    // The font is one big texture!
    unsigned int texture_id;
};

// Loads a font from a TTF file to our internal font representation.
void load_font(Font* font, const char* file_name) {
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
    //      2) the pixel height of the font. This is hardcoded! We want to pick something reasonably large so that we never
    //         have to scale it up (that would make the text look pixelated). In order to have perfect rendering at every
    //         text size, we would have to use something called signed distance fields, which are really cool but would
    //         require using OpenGL 4, which is overcomplicated for our purposes.
    //      3) the memory we allocated for the bitmap.
    //      4) the width of the bitmap.
    //      5) the height of the bitmap.
    //      6) the character code of the first character we want to support. We want to just render the visible ASCII characters,
    //         which start at 32.
    //      7) the number of characters we want to support. We want all from 32 until the second to last signed ASCII character.
    //      8) an array in which information about each character is stored (the location of the character in the bitmap).
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

    glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE_ALPHA, 1024, 1024, 0, GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE, real_font_bitmap);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
}

// Returns the width and height of the given text if it were rendered with the given font.
void size_text(Font* font, const char* text, int* width, int* height) {
    // Basic idea: iterate through each character in text, and check its height.
    // Check to see if it has the greatest height of all characters we want to display.
    // Then the rightmost x coordinate of the last character is the width of the whole string.

    float max_height = 0;
    float total_width = 0;
    {
        float x = 0, y = 0;
        for (const char* text_ptr = text; *text_ptr; text_ptr++) {
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

    *width = (int) total_width;
    *height = (int) max_height;
}

int draw_text(Font* font, const char* text, int x, int y, float height) {
    // This first part is just calculating the max height of all characters to be rendered.
    // This is required because the y component passed to this function specifies the position
    // of the top left corner of the text, but characters are aligned at the baseline (bottom).

    float max_height = 0;
    {
        float x = 0, y = 0;
        for (const char* text_ptr = text; *text_ptr; text_ptr++) {
            stbtt_aligned_quad q;
            stbtt_GetBakedQuad(font->baked_chars, 1024, 1024, *text_ptr - 32, &x, &y, &q, 1);

            if (q.y1 - q.y0 > max_height) {
                max_height = q.y1 - q.y0;
            }
        }
    }

    // We want the tallest character to have the height we specified.
    // That way, the string will be at most height pixels tall.
    // This works because scale will be multiplied by each character height.
    // When the tallest character comes along, we have max_height * (height / max_height)
    // which simplifies to height.
    float scale = height / max_height;

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

    return (int)max_height;
}

// Helper method for addMessage
void removeOldMessages(){
	while(logMessages.size() > MAX_LINES){
		logMessages.erase(logMessages.begin());
		red.erase(red.begin());
		green.erase(green.begin());
		blue.erase(blue.begin());
		alpha.erase(alpha.begin());
	}
}

// addMessage("Message Here", red, green, blue, alpha)
void addMessage(std::string m, float r, float g, float b, float a){
	while(m.length() > 0){
		if(m.length() <= MAX_CHARS_IN_A_LINE){
			logMessages.push_back(m);
			red.push_back(r);
			green.push_back(g);
			blue.push_back(b);
			alpha.push_back(a);
			break;
		}
		else{
			logMessages.push_back(m.substr(0, MAX_CHARS_IN_A_LINE));
			red.push_back(r);
			green.push_back(g);
			blue.push_back(b);
			alpha.push_back(a);
			m.erase(0, MAX_CHARS_IN_A_LINE);
		}
	}
	removeOldMessages();
}

// Test method to make sure the log is working properly
void testLog() {
    addMessage("Hello World!", 1.0f, 1.0f, 1.0f, 1.0f);
    addMessage("Hello World!", 1.0f, 1.0f, 1.0f, 1.0f);
    addMessage("Hello World!", 1.0f, 1.0f, 1.0f, 1.0f);
    addMessage("Hello World!", 1.0f, 1.0f, 1.0f, 1.0f);
    addMessage("I'm BLUE da ba dee da ba die, da ba dee, da ba die, da ba dee da ba die!", 0.0f, 0.0f, 1.0f, 1.0f);
    addMessage("Hello World!", 1.0f, 1.0f, 1.0f, 1.0f);
    addMessage("Error: The rover is literally on fire oh god oh geez oh no", 1.0f, 0.0f, 0.0f, 1.0f);
    addMessage("Hello World!", 1.0f, 1.0f, 1.0f, 1.0f);
    addMessage("Hello World!", 1.0f, 1.0f, 1.0f, 1.0f);
    addMessage("Life: Exists", 0.0f, 1.0f, 1.0f, 1.0f);
    addMessage("Greek Philosophers: HmmmmMMMMMMMMmm", 0.0f, 1.0f, 1.0f, 1.0f);
    addMessage("Hello World!", 1.0f, 1.0f, 1.0f, 1.0f);
}

int main() {
	// Start the timer.
	start_time = std::chrono::high_resolution_clock::now();

	// Init GLFW.
    if (!glfwInit()) {
        fprintf(stderr, "[!] Failed to init GLFW!\n");
        return 1;
    }

    // Init the controller.
    bool controller_loaded = false;
    if (controller::init("/dev/input/js0") == controller::Error::OK) {
        controller_loaded = true;
    } else {
        printf("No controller.\n");
    }

    // Fill the Log with test messages
    testLog();


    // Create a fullscreen window. Title isn't displayed, so doesn't really matter.
	GLFWwindow* window = glfwCreateWindow(gui::WINDOW_WIDTH, gui::WINDOW_HEIGHT, "Base Station", glfwGetPrimaryMonitor(), NULL);

	// Update the window so everyone can access it.
	gui::state.window = window;

	// Set sticky keys mode. It makes our input work as intended.
	glfwSetInputMode(window, GLFW_STICKY_KEYS, 1);

	glfwSetCharCallback(window, glfw_character_callback);
	glfwSetKeyCallback(window, glfw_key_callback);

    // Create an OpenGL context.
	glfwMakeContextCurrent(window);

    // OpenGL Setup.
    glViewport(0, 0, gui::WINDOW_WIDTH, gui::WINDOW_HEIGHT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, gui::WINDOW_WIDTH, gui::WINDOW_HEIGHT, 0, 0, 0.5);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Initialize camera stuff.
    camera_feed::init();

    // Create the camera streams.
    camera_feed::Feed feeds[4];
    camera_feed::init_feed(&feeds[0], 1920, 1080);
    camera_feed::init_feed(&feeds[1], 1920, 1080);
    camera_feed::init_feed(&feeds[2], 1920, 1080);
    camera_feed::init_feed(&feeds[3], 1920, 1080);

    // Initialize network functionality.
    network::Connection conn;
    {
        network::Error err = network::connect(&conn, "127.0.0.1", 45546, 45545);
        if (err != network::Error::OK) {
            fprintf(stderr, "[!] Failed to connect to rover!\n");
            return 1;
        }
    }

    // Init the font
    Font font;
    load_font(&font, "F25_Bank_Printer.ttf");

    // Keep track of when we last sent movement info.
    unsigned int last_movement_send_time = 0;

	gui::Font debug_console_font;
	bool loaded_font = gui::load_font(&debug_console_font, "res/FiraMono-Regular.ttf", 100);
	if (!loaded_font) {
		fprintf(stderr, "[!] Failed to load debug console font!\n");
		return 1;
	}

	gui::debug_console::log("Debug log initialized.", 0, 1.0, 0);

    while (!glfwWindowShouldClose(window)) {
		glfwPollEvents();

		if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
			if (!gui::state.show_debug_console) {
				gui::state.show_debug_console = true;
				gui::state.input_state = gui::InputState::DEBUG_CONSOLE;
			}
		}

        // Handle incoming network messages.
        network::poll_incoming(&conn);
        
        network::Message message;
        while (network::dequeue_incoming(&conn, &message)) {
            switch (message.type) {
                case network::MessageType::HEARTBEAT: {
                    network::Buffer* outgoing = network::get_outgoing_buffer();
                    network::queue_outgoing(&conn, network::MessageType::HEARTBEAT, outgoing);
                    break;
                }
                case network::MessageType::CAMERA: {
                    // Static buffer so we don't have to allocate and reallocate every frame.
                    static uint8_t camera_message_buffer[CAMERA_MESSAGE_FRAME_DATA_MAX_SIZE];

                    network::CameraMessage camera_message;
                    camera_message.data = camera_message_buffer;
                    network::deserialize(message.buffer, &camera_message);

                    if (camera_message.stream_index > 4) {
                        break;
                    }

                    camera_feed::Error err = camera_feed::handle_section(&feeds[camera_message.stream_index], camera_message.data, camera_message.size, camera_message.section_index, camera_message.section_count, camera_message.frame_index);
                    if (err != camera_feed::Error::OK) {
                        fprintf(stderr, "[!] Failed to handle frame section!\n");
                    }

                    break;
                }
                default:
                    break;
            }

			network::return_incoming_buffer(message.buffer);
        }

        if (controller_loaded) {
            // Process controller input.
            controller::Event event;
            controller::Error err;
            
            // Do nothing since we just want to update current values.
            while ((err = controller::poll(&event)) == controller::Error::OK) {}

            if (err != controller::Error::DONE) {
                fprintf(stderr, "[!] Failed to read from the controller! Disabling.\n");
                controller_loaded = false;
            } else {
                if (get_ticks() - last_movement_send_time >= MOVMENT_SEND_INTERVAL) {
                    last_movement_send_time = get_ticks();

                    network::Buffer* message_buffer = network::get_outgoing_buffer();

                    network::MovementMessage message;
                    message.left = -controller::get_value(controller::Axis::JS_LEFT_Y);
                    message.right = -controller::get_value(controller::Axis::JS_RIGHT_Y);
                    network::serialize(message_buffer, &message);

                    network::queue_outgoing(&conn, network::MessageType::MOVEMENT, message_buffer);
                }
            }
        }

        // Update and draw GUI.
        do_gui(feeds, &debug_console_font);
	glColor4f(0.0f, 0.0f, 0.0f, 1.0f);

	//Draw log text now
	for(unsigned int i = 0; i < logMessages.size(); i++){
		const char *cstr = logMessages.at(i).c_str();
		glColor4f(red.at(i), green.at(i), blue.at(i), alpha.at(i));
		draw_text(&font, cstr, LOG_X + LOG_THICKNESS + 5, LOG_Y + LOG_THICKNESS + 5 + 30*i, 20);
	}

        // Display our buffer.
	glfwSwapBuffers(window);

        // Send any messages that we accumulated.
        network::drain_outgoing(&conn);
    }

    // Cleanup.
	glfwTerminate();

    return 0;
}
