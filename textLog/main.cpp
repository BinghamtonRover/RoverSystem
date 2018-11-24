// You must install the libgl1-mesa-dev and libsdl2-dev packages (via apt-get) first!

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <vector>
#include <SDL.h>
#include <GL/gl.h>
#include <tuple>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#define STB_TRUETYPE_IMPLEMENTATION
#include "stb_truetype.h"

const int WIDTH = 640;
const int HEIGHT = 480;

struct Font {
    stbtt_bakedchar baked_chars[95];

    unsigned int texture_id;
};

void load_font(Font* font, const char* file_name) {
    FILE* font_file = fopen(file_name, "r");

    static unsigned char font_buffer[1 << 20];

    fread(font_buffer, 1, 1 << 20, font_file);

    fclose(font_file);

    static unsigned char font_bitmap[1024 * 1024];

    if (stbtt_BakeFontBitmap(font_buffer, 0, 50, font_bitmap, 1024, 1024, 32, 127 - 32, font->baked_chars) < 0) {
        fprintf(stderr, "[!] Failed to load font bitmap!\n");
        exit(1);
    }

    static unsigned char real_font_bitmap[1024 * 1024 * 2];
    
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

void size_text(Font* font, const char* text, int* width, int* height) {
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

    float scale = height / max_height;

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glScalef(scale, scale, scale);

    // Must be called before glBegin().
    glBindTexture(GL_TEXTURE_2D, font->texture_id);

    glEnable(GL_TEXTURE_2D);
    glBegin(GL_QUADS);
    {
        float tx = x, ty = y + max_height;

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

    glPopMatrix();

    return (int)max_height;
}

class TextLog {

public:
    std::vector< std::tuple <const char *,float,float,float, int, int> > entries;
    int logWidth;
    int logHeight;
    int heightSize;
    Font textFont;

    TextLog(int width,int height,const char * font){
	this->logWidth = width;
	this->logHeight = height;
	this->heightSize = 0;
    	load_font(&this->textFont, font);
    }

    void addEntry(const char * text,float red,float green, float blue){
	std::tuple <const char *,float,float,float,int,int> entry;
	int width = 0;
	int height = 0;
	size_text(&this->textFont,text,&width,&height);
	entry = std::make_tuple(text,red,green,blue,width,height);
	entries.emplace(this->entries.begin(),entry);
	this->heightSize += height;
	while (this->heightSize > this->logHeight) {
	    int lastEntry = this->entries.size()-1;
	    heightSize -= std::get<5>(this->entries[lastEntry]);
	    this->entries.erase(this->entries.end()-1);
	}
    
    }

    void printEntries(){
	int heightPos = 0;
	if (!this->entries.empty()) //To take care of the case where nothing is in our textlog
	    heightPos = std::get<5>(this->entries[0]);

        for(auto it = this->entries.begin(); it != this->entries.end(); it++){
	    const char * text = std::get<0>(*it);
	    float red = std::get<1>(*it);
	    float green = std::get<2>(*it);
	    float blue = std::get<3>(*it);
	    int width = std::get<4>(*it);
	    int height = std::get<5>(*it);
            glColor4f(red,green,blue,1.0f);
	    draw_text(&this->textFont,text,0,this->logHeight-heightPos,height);
	    heightPos += height;
	}


    }

};


unsigned int load_texture(const char* file_name) {
    int texture_w, texture_h, texture_channels;
    unsigned char* texture_data = stbi_load(file_name, &texture_w, &texture_h, &texture_channels, 3);

    unsigned int tex_id;
    glGenTextures(1, &tex_id);

    glBindTexture(GL_TEXTURE_2D, tex_id);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texture_w, texture_h, 0, GL_RGB, GL_UNSIGNED_BYTE, texture_data);

    stbi_image_free(texture_data);

    return tex_id;
}



int main() {
    SDL_Init(SDL_INIT_VIDEO);
    
    SDL_Window* window = SDL_CreateWindow("TextLog Demo", 0, 0, 640, 480, SDL_WINDOW_OPENGL);

    SDL_GLContext gl_context = SDL_GL_CreateContext(window);
    SDL_GL_MakeCurrent(window, gl_context);

    printf("OpenGL Version: %s\n", glGetString(GL_VERSION));

    SDL_GL_SetSwapInterval(1);

    glViewport(0, 0, WIDTH, HEIGHT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, WIDTH, HEIGHT, 0, 0, 0.5);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    TextLog log = TextLog(640,480,"Niramit-Regular.ttf");

    bool running = true;
    while (running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running = false;
                break;
            }
	    if (event.type == SDL_KEYDOWN) {
		    if (event.key.keysym.sym == SDLK_UP) {
			log.addEntry("UP KEY was pressed!",1.0f,0.0f,0.0f);
		    }

		    if (event.key.keysym.sym == SDLK_DOWN) {
			log.addEntry("DOWN KEY was pressed!",0.0f,1.0f,0.0f);
		    }
		    
		    if (event.key.keysym.sym == SDLK_LEFT) {
			log.addEntry("LEFT KEY was pressed!",0.0f,0.0f,1.0f);
		    }
		    if (event.key.keysym.sym == SDLK_RIGHT) {
			log.addEntry("RIGHT KEY was pressed!",1.0f,0.0f,1.0f);
		    }
	    }

        }

        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

	log.printEntries();

        SDL_GL_SwapWindow(window);
    }

    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
