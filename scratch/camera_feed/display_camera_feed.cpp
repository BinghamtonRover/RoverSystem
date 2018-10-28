#include "../../src/network/network.hpp"
#include "display_camera_feed.hpp"
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <turbojpeg.h>
#include "SDL2/SDL.h"
#include <inttypes.h> //to print uints

#define CAMERA_MESSAGE_FRAME_DATA_MAX_SIZE 65000
#define CAMERA_FRAME_BUFFER_SIZE 6220800
#define CAMERA_FRAME_BUFFER_COUNT 5

void decompress(unsigned char* buffer, unsigned char* compressed_image, int pitch, uint32_t size){
		
		if(compressed_image[CAMERA_FRAME_BUFFER_SIZE] == '\0')
				printf("Ending character present.\n");
		std::cout << "decompressing" << std::endl;
		int  width = 1920, height = 1080 ;
		long unsigned int _jpegSize = width * height * 3;
		tjhandle _jpegDecompressor = tjInitDecompress();
		if(_jpegDecompressor != NULL)
				std::cout << "decompressor was made" << std::endl;

		//The parameters are the decompressor, the compressed image, the size of the
		//image as an unsigned long, the buffer, the width of the jpeg, the pitch,
		//the height, the pixel type (We have RGB here each contained in 1 byte),
		//and the flags. Basically we say decompress it quickly.
		printf("Compressed image ptr: %p", compressed_image);
	    std::cout << ". JPG size: " << _jpegSize <<  ". Pitch " << pitch << std::endl;
		printf(". Buffer ptr %p\n", buffer); 
		if (tjDecompress2(_jpegDecompressor, compressed_image, _jpegSize, buffer, width, pitch, height, TJPF_RGB, TJFLAG_FASTDCT) != -1)
				std::cout << "decompressed correctly" << std::endl;
		else
				std::cout << "decompressed incorrectly" << std::endl;
		//free the decompressor.
		tjDestroy(_jpegDecompressor);
}

//method to get the size from the byte buffer.

int main(){
		std::cout << "Starting" << std::endl;
		//create a 2d array of buffers.
		int pitch = 1920*3;
		unsigned char* temp_compressed_message_buffer = new unsigned char[CAMERA_FRAME_BUFFER_SIZE];
		BufferItem* frame_buffer = new BufferItem[CAMERA_FRAME_BUFFER_COUNT];
		unsigned char* pixel_location;
		//
		for(int i = 0; i < CAMERA_FRAME_BUFFER_COUNT; i++){
				frame_buffer[i].data_sections = new unsigned char[CAMERA_FRAME_BUFFER_SIZE];
		}
		uint8_t indx = 0;
		//establish a newtwork connection
		network::Connection conn;
		//establish network error
		const char* destination_port = "127.0.0.1";
		network::Error error = network::connect(&conn, destination_port, 45546, 45545) ;
		if(error == network::Error::OK)
				std::cout << "Error bound a port correctly" << std::endl;
		else if(error == network::Error::OPEN_SOCKET)
				std::cout << "Open socket error: " << std::endl;
		else if(error == network::Error::BIND_SOCKET)
				std::cout << "Binding socket error" << std::endl;
		//initiate SDL, if its 0 something went wrong. Then we print the err
		if (SDL_Init(SDL_INIT_VIDEO) != 0){
				std::cout << "SDL_Init Error: " << SDL_GetError() << std::endl;
				return 1;
		}
		//creates window. pramaeters are window title, x start pos, y start pos,
		//x end pos, and y end pos.
		SDL_Window *win = SDL_CreateWindow("Hello World!", 0, 0, 1920, 1080, SDL_WINDOW_SHOWN);
		//check for failure
		if (win == nullptr){
				std::cout << "SDL_CreateWindow Error: " << SDL_GetError() << std::endl;
				SDL_Quit();
				return 1;
		}
		//makes a renderer. args are the window to render to, the index of the 
		//rendering driver, and the rendering flags or'd together
		//SDL ACCELERATED uses hardware acceleration, and SDL PRESENTVSYNC
		//is present is synchronized with the refresh rate
		//Other flags are TARGETTEXTURE -- supports redering to texture
		// and SOFTWARE -- which means the renderer is a software fallback
		SDL_Renderer *ren = SDL_CreateRenderer(win,-1,
						SDL_RENDERER_ACCELERATED   | SDL_RENDERER_PRESENTVSYNC);
		if (ren == nullptr){
				SDL_DestroyWindow(win);
				std::cout << "SDL_CreateRenderer Error: " << SDL_GetError() << std::endl;
				SDL_Quit();
				return 1;
		}
		SDL_Texture *texture = SDL_CreateTexture(ren, SDL_PIXELFORMAT_RGB24, SDL_TEXTUREACCESS_STREAMING, 1920, 1080);
		std::cout << "Initialized everything" << std::endl;
		std::cout << "Starting Loops" << std::endl;
		bool running = true;
		SDL_Event event;
		while(running){
				 if (SDL_PollEvent(&event)) {
  	 		  	 	switch (event.type) {
					case SDL_QUIT:
						running = false;
						break;
				 	}
				 }
				//polls for incoming messages
				network::poll_incoming(&conn);
				//declares a message to hold the incoming data
				network::Message message;
				//while there are more messages
				while(network::dequeue_incoming(&conn, &message)) {
						std::cout << "dequeueing messages" << std::endl;
						//get the mesage type
						switch (message.type) {
								case network::MessageType::CAMERA: {
										std::cout << "recieved CameraMessage" << std::endl;
										network::CameraMessage frame{ 0 };
										std::cout << "Created the frame" << std::endl;
										frame.data = temp_compressed_message_buffer;
										//deserialize the message into the cameraMessage
										network::deserialize(message.buffer, &frame);
										std::cout << "deserialized message, Size: " << frame.size << std::endl;
										std::cout << "deserialized message index: " << frame.frame_index << std::endl;
										//get the index of the buffer that the frame is supposed to go into
										indx = frame.frame_index % CAMERA_FRAME_BUFFER_COUNT;
										if(frame_buffer[indx].frame_index != frame.frame_index){
												frame_buffer[indx].frame_index = frame.frame_index;
												std::cout << "Section count: " << unsigned(frame.section_count) << std::endl;
												printf("%" PRIu16 "\n", frame_buffer[indx].sections_remaining);
												frame_buffer[indx].frame_index = frame.frame_index;
												frame_buffer[indx].sections_remaining = frame.section_count;
												frame_buffer[indx].data_size = 0;
												std::cout << "Initial Sections Remaining: " << unsigned(frame_buffer[indx].sections_remaining) << std::endl;
												std::cout << "data is 0. Proof: " << unsigned(frame_buffer[indx].data_size) << std::endl;
										}
										frame_buffer[indx].data_size += frame.size;
										std::cout << "New data: " << unsigned(frame_buffer[indx].data_size) << std::endl;
										frame_buffer[indx].sections_remaining--;
										std::cout << "buffer remaining: " << unsigned(frame_buffer[indx].sections_remaining) << 	std::endl;
										std::cout << "Section index: " << unsigned(frame.section_index) << 	std::endl;
										//TODO check if the frame full if it is a new index.
										//if it is we unlock the pixel buffer,
										//decompress the JPEG stored into a pixels buffer to be displayed,
										//and then lock it
										//here we access the buffer at the frame % buffer count index.
										//We then increment that index to the start of the section index.
										//the +5 here is to get past the first 5 bytes which are the 
										//frame index and the remaining number of sections to be recieved respectively
										//TODO make sure below code is how Layne wants it done
										std::memcpy(frame_buffer[indx].data_sections+ (CAMERA_MESSAGE_FRAME_DATA_MAX_SIZE * frame.section_index), frame.data, frame.size);
										//decompress the buffer
										//check if we have a full frame
										std::cout << "Remaining: " << unsigned(frame_buffer[indx].sections_remaining) << std::endl;
										if(frame_buffer[indx].sections_remaining  == 0){
												std::cout << "We have a full frame" << std::endl;
												std::cout << "Remaining: " << unsigned(frame_buffer[indx].data_size) << std::endl;
												//if we do decompress the buffer
												//Lock texture to allow writing to pictures
												SDL_LockTexture( texture,NULL, (void **) &pixel_location, &pitch);
												decompress(( pixel_location) , frame_buffer[indx].data_sections, pitch, frame_buffer[indx].data_size);
												//unlock texture to update it again.
												SDL_UnlockTexture(texture);
												//actually render texture in for loop below
												//This is what the default display for the renderer does, not sure
												//if its needed.
												SDL_RenderClear(ren);
												//Draw the texture
												//The parameters are the renderer, texture, and the borders to display it
												// (In sdl windows). When these parameters are null it displays fully on
												//the window provided
												SDL_RenderCopy(ren, texture, NULL, NULL);
												//Updates the screen based on the renderer
												SDL_RenderPresent(ren);
										}
										break;
								}
								default:
										std::cout << "Non CameraMessage message recieved" << std::endl;
										//deserialize the message into the buffer.
										//Decompress image into buffer

						}
                network::return_incoming_buffer(message.buffer);
				}
		}
		//free all the SDL parts to avoid memory leaks
		SDL_DestroyTexture(texture);
		SDL_DestroyRenderer(ren);
		SDL_DestroyWindow(win);
		SDL_Quit();
}
