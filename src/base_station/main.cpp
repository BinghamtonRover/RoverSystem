#include <stdio.h>
#include <stdlib.h>

#include <SDL.h>

// We know that our base station will have this resolution.
const int WINDOW_WIDTH = 1920;
const int WINDOW_HEIGHT = 1080;

int main() {
    // Init just the video subsystem of SDL.
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        fprintf(stderr, "[!] Failed to init SDL: %s\n", SDL_GetError());
        return 1;
    }

    // Create a fullscreen window. Title isn't displayed, so doesn't really matter.
    SDL_Window* window = SDL_CreateWindow("Base Station", 0, 0, WINDOW_WIDTH, WINDOW_HEIGHT, SDL_WINDOW_FULLSCREEN);

    // Create a renderer so we actually see something.
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

    bool running = true;
    while (running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            switch (event.type) {
            case SDL_QUIT:
                running = false;
                break;
            case SDL_KEYDOWN:
                if (event.key.keysym.sym == SDLK_ESCAPE)
                    running = false;
                break;
            }
        }

        // Early exit if we just decided to quit.
        if (!running) break;

        // Clear the screen to white.
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
        SDL_RenderClear(renderer);

        // ... do other drawing here ...

        // Display our buffer.
        SDL_RenderPresent(renderer);
    }

    // Cleanup.
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}