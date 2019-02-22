#include "../../src/network/network.hpp"
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <turbojpeg.h>
#include <SDL2/SDL.h>

struct BufferItem {
    uint16_t frame_index;
    uint16_t data_size;
    uint8_t sections_remaining;
    unsigned char* data_sections;
};

const int CAMERA_MESSAGE_FRAME_DATA_MAX_SIZE  = 65000;
const int CAMERA_FRAME_BUFFER_SIZE = 6220800;
const int CAMERA_FRAME_BUFFER_COUNT = 5;
/*
 This will change the size of the window rendered.
 Standard Resolution is 1920 by 1080
 */
const int WINDOW_X_LENGTH = 1920;
const int WINDOW_Y_LENGTH = 1080;
const long unsigned int JPEG_SIZE = WINDOW_X_LENGTH * WINDOW_Y_LENGTH * 3;
const char* DESTINATION_ADDRESS = "127.0.0.1";
tjhandle jpeg_decompressor;

void decompress(unsigned char* buffer, unsigned char* compressed_image, int pitch){
    /*
        The parameters are the decompressor, the compressed image, the size of the
        image as an unsigned long, the buffer, the width of the jpeg, the pitch,
        the height, the pixel type (We have RGB here each contained in 1 byte),
        and the flags. Basically we say decompress it quickly.
    */
    if (tjDecompress2(jpeg_decompressor, compressed_image, JPEG_SIZE, buffer, 
    WINDOW_X_LENGTH, pitch, WINDOW_Y_LENGTH, TJPF_RGB, TJFLAG_FASTDCT) == -1){
        std::cout << "decompressed incorrectly" << std::endl;
    }
}
/*
    If there are no sections remaining, lock the pixels 
    for writing, and pass the location into decompress. 
    Decompress writes the decompressed pixel data to 
    pixel_location. Then we unlock the texture again 
    to render it to the screen.
 */
void setUpFrame(BufferItem* frame_buffer, network::CameraMessage &frame, uint8_t &current_frame_indx){
/*
    Since we know that camera packets will only be sent out once we
    can manage the buffer by only keeping track of the number of 
    sections needed. We get the index by taking the frame index mod
    the total number of buffers.
 */
    current_frame_indx = frame.frame_index % CAMERA_FRAME_BUFFER_COUNT;
    /*
        If we have done a full loop and there is a new
        frame index, then we overwrite the old data.
     */
    if(frame_buffer[current_frame_indx].frame_index != frame.frame_index){
        frame_buffer[current_frame_indx].frame_index = frame.frame_index;
        frame_buffer[current_frame_indx].sections_remaining = frame.section_count;
        frame_buffer[current_frame_indx].data_size = 0;
    }
    frame_buffer[current_frame_indx].data_size += frame.size;
    frame_buffer[current_frame_indx].sections_remaining--;
    /*
        This section handles any data that comes out of order.
        Each data section is written at the size of
        a signle data packet times the section index.
     */
    uint32_t offset = (CAMERA_MESSAGE_FRAME_DATA_MAX_SIZE * frame.section_index);
    std::memcpy(frame_buffer[current_frame_indx].data_sections + offset , frame.data, frame.size);

}

void renderFrame(SDL_Texture* texture, SDL_Renderer* ren, unsigned char* pixel_location,  unsigned char* data_sections){
    /*
        Pitch is the number of bytes in a single row of JPEG data. 
        Due to hardware padding this is not always width * numBytes per RGB value.
     */
    int pitch;
    SDL_LockTexture(texture, NULL, (void **) &pixel_location, &pitch);
    decompress(pixel_location , data_sections, pitch);
    SDL_UnlockTexture(texture);
    //This clears anything currently in the renderer
    SDL_RenderClear(ren);
    /*
        This loads the texture onto the renderer.
        The NULL parameters specify the demensions
        of the window to render to. Null renders to 
        the full extent of the window.
     */
    SDL_RenderCopy(ren, texture, NULL, NULL);
    //This renders the contents of the renderer.
    SDL_RenderPresent(ren);
}

int main(){
    jpeg_decompressor = tjInitDecompress();;
    if (!jpeg_decompressor) {
        std::cout << "Error creating decompressor!" << std::endl;
        return 1;
    }

    BufferItem* frame_buffer = new BufferItem[CAMERA_FRAME_BUFFER_COUNT];
    /*
        Allocates space for frame data within each frame buffer.
     */
    for(int i = 0; i < CAMERA_FRAME_BUFFER_COUNT; i++){
        frame_buffer[i].data_sections = new unsigned char[CAMERA_FRAME_BUFFER_SIZE];
    }
    /*
        We need a single additional buffer to store
        the data we recieve from the network.
     */
    unsigned char* temp_compressed_message_buffer = new unsigned char[CAMERA_FRAME_BUFFER_SIZE];
    uint8_t current_frame_indx;

    /*
        We have to write the pixel data to a specific location,
        so we need a pointer to store that location
     */
    unsigned char* pixel_location;
    /*
        Pitch is the number of bytes in a row of an image.
        (This value can change if the hardware uses padding)
     */
    network::Connection conn;
    //This will allow error to fade out of scope after we no longer need it.
    {
        network::Error error = network::connect(&conn, DESTINATION_ADDRESS, 45546, 45545) ;
        if(error == network::Error::OPEN_SOCKET){
            std::cout << "Error opening the socket." << std::endl;
        }
        else if(error == network::Error::BIND_SOCKET){
            std::cout << "Error binding the socket" << std::endl;
        }
    }
    /*
        Here we ckeck if any of the SDL components are
        created incorrectly. If they fail we return from main
        because we cannot display anything.
     */
    if (SDL_Init(SDL_INIT_VIDEO) != 0){
        std::cout << "SDL_Init Error: " << SDL_GetError() << std::endl;
        return 1;
    }
    /*
        This creates an SDL window. The parameters are the window title,
        x start pos and y start pos (both as an offset from the top
        left corner), x end pos, and y end pos. We use the dimensions
        of the window provided above
     */
    SDL_Window *win = SDL_CreateWindow("Camera feed!", 0, 0,
        WINDOW_X_LENGTH, WINDOW_Y_LENGTH, SDL_WINDOW_SHOWN);
    if (win == nullptr){
        std::cout << "SDL_CreateWindow Error: " << SDL_GetError() << std::endl;
        SDL_Quit();
        return 1;
    }
    /*
        This makes a renderer. The arguments are
        the window to render to, the index of the rendering driver,
        and the rendering flags or'd together
        SDL ACCELERATED uses hardware acceleration, and SDL PRESENTVSYNC
        synchronizes with the refresh rate
        Other flags are TARGETTEXTURE which supports redering to texture
        and SOFTWARE which means the renderer has a software fallback 
        should the hardware accelerated rendering fail
    */
    SDL_Renderer *ren = SDL_CreateRenderer(win,-1,
    SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (ren == nullptr){
        SDL_DestroyWindow(win);
        std::cout << "SDL_CreateRenderer Error: " << SDL_GetError() << std::endl;
        SDL_Quit();
        return 1;
    }
     /*
        This makes a texture. The arguments are
        the renderer, the pixel storage format (3 bytes in our case)
        and the rendering flags or'd together
        here we only use TEXTUREACCESS_STREAMING, which allows for 
        the texture to change repeatedly.
    */
    SDL_Texture *texture = SDL_CreateTexture(ren, SDL_PIXELFORMAT_RGB24, 
    SDL_TEXTUREACCESS_STREAMING, 1920, 1080);
    bool running = true;
    SDL_Event event;
    while(running){
        //If someone clicks the x, exit the program.
        if (SDL_PollEvent(&event)) {
            if(event.type == SDL_QUIT) {
                    running = false;
            }
        }
        //This is the standard network protocall to recieve a packet.
        network::poll_incoming(&conn);
        network::Message message;
        while(network::dequeue_incoming(&conn, &message)) {
            if(message.type == network::MessageType::CAMERA) {
                //We need a CameraMessage to hold the packet data.
                network::CameraMessage frame{};
                /*
                    Here we point the new message data
                    to our temporary buffer.
                 */
                frame.data = temp_compressed_message_buffer;
                network::deserialize(message.buffer, &frame);
                //This manages the frame buffer, and aligns data.
                setUpFrame(frame_buffer, frame, current_frame_indx);
                if(frame_buffer[current_frame_indx].sections_remaining  == 0){
                    //This renders the frame to the window.
                    renderFrame(texture, ren, pixel_location, 
                    frame_buffer[current_frame_indx].data_sections);
                }
            }
            else {
                std::cout << "Non CameraMessage message recieved" << std::endl;
            }
        network::return_incoming_buffer(message.buffer);
        }
    }
    /*
        We need to free the decompressor and
        SDL components to avoid memory leaks
     */
    tjDestroy(jpeg_decompressor);
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
