#ifndef DISPLAY_CAMERA_FEED_HPP
#define DISPLAY_CAMERA_FEED_HPP
int main ();

void decompress(unsigned char* buffer, unsigned char* compressed_image, int pitch);
//Bytebuffer[size] [0-3] frame index. [4] remaining.
struct BufferItem{
		uint16_t frame_index;
		uint16_t data_size;
		uint8_t sections_remaining;
		unsigned char* data_sections;
};
#endif
