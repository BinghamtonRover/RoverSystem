// You must install the libgl3-mesa-dev and libsdl2-dev packages (via apt-get) first!

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <vector>
#include <SDL.h>
#include <GL/gl.h>
#include <tuple>

#define STB_TRUETYPE_IMPLEMENTATION
#include "stb_truetype.h"

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

class textLog {
public:
    std::vector< std::tuple <const char *,float,float,float,int,int> > entries;
    std::vector< std::tuple <const char *,float,float,float,int,int> > savedEntries;
    int logWidth;
    int logHeight;
    int curPageHeight;
    int totalHeight;
    Font textFont;
    int currentPage;
    int entriesPerPage;
    int totalPages;

    textLog(int width,int height,const char * font){
        this->logWidth = width;
        this->logHeight = height;
        this->curPageHeight = 0;
        this->totalHeight = 0;
        this->currentPage = 1;
        this->entriesPerPage = 0;
        this->totalPages = 1;
        load_font(&this->textFont, font);
    }
    
    void addEntry(const char * text,float red,float green, float blue){
        std::tuple <const char *,float,float,float,int,int> entry;
        int width = 0;
        int height = 0;
        size_text(&this->textFont,text,&width,&height);
        entry = std::make_tuple(text,red,green,blue,width,height);
        entries.emplace(this->entries.begin(),entry);
        this->curPageHeight += height;
        this->totalHeight += height;
        while (this->curPageHeight > this->logHeight) {
            if (this->entriesPerPage == 0) { this->entriesPerPage = this->savedEntries.size(); } //This is to set the number of entries per page
            std::tuple <const char *,float,float,float,int,int> lastEntry = *(this->entries.end()-1);
            this->curPageHeight -= std::get<5>(lastEntry);
            this->entries.erase(this->entries.end()-1);
        }
        savedEntries.emplace(this->savedEntries.begin(),entry);
    
    }
    
    void printEntries(){
        generatePages();
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
    
    void generatePages(){
        this->totalPages = (this->totalHeight/this->logHeight)+1; //The plus 1 for the fact that there should always be 1 page
        if ( (this->totalHeight % this->logHeight) == 0 ) { totalPages--; } //This is for the case where there are exactly enough entries to fill a page  
        if (this->totalPages > 1) {
            this->entries.clear();   
            int indexOfCurPage = (this->currentPage * this->entriesPerPage)-this->entriesPerPage; //Subtracting entriesPerPage to deal with the offset because we aren't using 0 to initialize currentPage
            int numOfEntries = 0;
            while(indexOfCurPage < this->savedEntries.size() && numOfEntries < this->entriesPerPage) {
	        this->entries.push_back(*(this->savedEntries.begin()+indexOfCurPage));
	        indexOfCurPage++;
	        numOfEntries++;
            }
        }
        else { this->entries = this->savedEntries; }
    }
    
    void pageUp() {
        if (this->totalPages > 1 && this->currentPage < this->totalPages){
            this->currentPage++;
        }
    }
    
    void pageDown() {
        if (this->totalPages > 1 && this->currentPage > 1){
            this->currentPage--;        
        }
    
    }

};


int WIDTH = 640;
int HEIGHT = 480;

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

    textLog log = textLog(640,480,"Niramit-Regular.ttf");

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
                if (event.key.keysym.sym == SDLK_PAGEUP) {
                    log.pageUp();
                }
                if (event.key.keysym.sym == SDLK_PAGEDOWN) {
                    log.pageDown();
                }
           }

        }

        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        log.printEntries();

        SDL_GL_SwapWindow(window);
    }

    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}

