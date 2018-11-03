#include "../../src/network/network.hpp"
#include "display_camera_feed.hpp"
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <turbojpeg.h>
#include "SDL2/SDL.h"
//#include <inttypes.h> to print uints

#define CAMERA_MESSAGE_FRAME_DATA_MAX_SIZE 65000
#define CAMERA_FRAME_BUFFER_SIZE 6220800
#define CAMERA_FRAME_BUFFER_COUNT 5
#define WINDOW_X_LENGTH 1920 //window resolution. Standard is 1920
#define WINDOW_Y_LENGTH 1080 //my window resolution. Standard is 1080

const long unsigned int _jpegSize = WINDOW_X_LENGTH * WINDOW_Y_LENGTH * 3;

void decompress(unsigned char* buffer, unsigned char* compressed_image, int pitch){
    tjhandle _jpegDecompressor = tjInitDecompress();
    if(_jpegDecompressor == NULL)
        std::cout << "decompressor was not made" << std::endl;
    /*
        The parameters are the decompressor, the compressed image, the size of the
        image as an unsigned long, the buffer, the width of the jpeg, the pitch,
        the height, the pixel type (We have RGB here each contained in 1 byte),
        and the flags. Basically we say decompress it quickly.
    */
    //std::cout << ". JPG size: " << _jpegSize <<  ". Pitch " << pitch << std::endl;
    if (tjDecompress2(_jpegDecompressor, compressed_image, _jpegSize, buffer, WINDOW_X_LENGTH, pitch, WINDOW_Y_LENGTH, TJPF_RGB, TJFLAG_FASTDCT) == -1)
        std::cout << "decompressed incorrectly" << std::endl;
    //free the decompressor.
    tjDestroy(_jpegDecompressor);
}

int main(){
<<<<<<< HEAD
    int pitch = 1920*3;
    uint32_t offset;
    //create a buffer for the message struct (declared in loop)
    unsigned char* temp_compressed_message_buffer = new unsigned char[CAMERA_FRAME_BUFFER_SIZE];
    //array of buffers to hold the compressed messages
    BufferItem* frame_buffer = new BufferItem[CAMERA_FRAME_BUFFER_COUNT];
    //pointer to where we keep the pixel data
    unsigned char* pixel_location;
    for(int i = 0; i < CAMERA_FRAME_BUFFER_COUNT; i++){
        frame_buffer[i].data_sections = new unsigned char[CAMERA_FRAME_BUFFER_SIZE];
    }
    uint8_t indx = 0;
    network::Connection conn;
    const char* destination_port = "127.0.0.1";
    //allow error to fade out of scope
    {
        network::Error error = network::connect(&conn, destination_port, 45546, 45545) ;
        if(error == network::Error::OPEN_SOCKET)
            std::cout << "Open socket error: " << std::endl;
        else if(error == network::Error::BIND_SOCKET)
            std::cout << "Binding socket error" << std::endl;
    }
    //check for failure
    if (SDL_Init(SDL_INIT_VIDEO) != 0){
        std::cout << "SDL_Init Error: " << SDL_GetError() << std::endl;
        return 1;
    }
    //creates window. pramaeters are window title, x start pos, y start pos,
    //x end pos, and y end pos. We use the dimensions of the window provided above
    SDL_Window *win = SDL_CreateWindow("Camera feed!", 0, 0, WINDOW_X_LENGTH, WINDOW_Y_LENGTH, SDL_WINDOW_SHOWN);
    //check for failure
    if (win == nullptr){
        std::cout << "SDL_CreateWindow Error: " << SDL_GetError() << std::endl;
        SDL_Quit();
        return 1;
    }
    /*
        makes a renderer. args are the window to render to, the index of the 
        rendering driver, and the rendering flags or'd together
        SDL ACCELERATED uses hardware acceleration, and SDL PRESENTVSYNC
        is present is synchronized with the refresh rate
        Other flags are TARGETTEXTURE -- supports redering to texture
        and SOFTWARE -- which means the renderer is a software fallback 
        should the hardware accelerated rendering fail
    */
    SDL_Renderer *ren = SDL_CreateRenderer(win,-1,
        SDL_RENDERER_ACCELERATED   | SDL_RENDERER_PRESENTVSYNC);
    if (ren == nullptr){
        SDL_DestroyWindow(win);
        std::cout << "SDL_CreateRenderer Error: " << SDL_GetError() << std::endl;
        SDL_Quit();
        return 1;
    }
    SDL_Texture *texture = SDL_CreateTexture(ren, SDL_PIXELFORMAT_RGB24, 
        SDL_TEXTUREACCESS_STREAMING, 1920, 1080);
    bool running = true;
    SDL_Event event;
    do{
        if (SDL_PollEvent(&event)) {
            switch (event.type) {
                case SDL_QUIT:
                    running = false;
                    break;
            }
        }
        network::poll_incoming(&conn);
        network::Message message;
        while(network::dequeue_incoming(&conn, &message)) {
            //std::cout << "dequeueing messages" << std::endl;
            switch (message.type) {
                case network::MessageType::CAMERA: {
                    network::CameraMessage frame{ 0 };
                    frame.data = temp_compressed_message_buffer;
                    network::deserialize(message.buffer, &frame);
                    //std::cout << "deserialized message, Size: " << frame.size << std::endl;
                    //std::cout << "deserialized message index: " << frame.frame_index << std::endl;
                    indx = frame.frame_index % CAMERA_FRAME_BUFFER_COUNT;
                    if(frame_buffer[indx].frame_index != frame.frame_index){
                        frame_buffer[indx].frame_index = frame.frame_index;
                        //std::cout << "Section count: " << unsigned(frame.section_count) << std::endl;
                        //printf(%" PRIu16 "\n", frame_buffer[indx].sections_remaining);
                        frame_buffer[indx].frame_index = frame.frame_index;
                        frame_buffer[indx].sections_remaining = frame.section_count;
                        frame_buffer[indx].data_size = 0;
                        //std::cout << "Initial Sections Remaining: " << unsigned(frame_buffer[indx].sections_remaining) << std::endl;
                    }
                    frame_buffer[indx].data_size += frame.size;
                    frame_buffer[indx].sections_remaining--;
                    //copy the frame data to our buffer (offset by the section index)
                    offset = (CAMERA_MESSAGE_FRAME_DATA_MAX_SIZE * frame.section_index);
                    std::memcpy(frame_buffer[indx].data_sections + offset , frame.data, frame.size);
                    //std::cout << "Remaining sections: " << unsigned(frame_buffer[indx].sections_remaining) << std::endl;
                    
                    if(frame_buffer[indx].sections_remaining  == 0){
                        /*
                            if there are no sections remaining,
                            lock the pixels for writing, pass the location into
                            decompress where the pixel data is written.
                            then unlock the texture again to render
                         */
                        SDL_LockTexture(texture, NULL, (void **) &pixel_location, &pitch);
                        decompress(pixel_location , frame_buffer[indx].data_sections, pitch);
                        SDL_UnlockTexture(texture);
                        //get rid of any garbage that may be in renderer
                        SDL_RenderClear(ren);
                        //load the texture onto the renderer. NULL renders to the full extent of the window
                        SDL_RenderCopy(ren, texture, NULL, NULL);
                        //render the contents of the renderer
                        SDL_RenderPresent(ren);
                    }
                    break;
                }
                default:
                    std::cout << "Non CameraMessage message recieved" << std::endl;
            }
            network::return_incoming_buffer(message.buffer);
        }
    }while(running);
    //free all the SDL parts to avoid memory leaks
    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();
    delete temp_compressed_message_buffer;
    for(int i = 0; i < CAMERA_FRAME_BUFFER_COUNT; i++){
        delete frame_buffer[i].data_sections;
    }
    delete frame_buffer;
    return 0;
}
