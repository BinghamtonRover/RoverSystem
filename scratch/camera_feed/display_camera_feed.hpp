
int main ( int argc, char *argv[] );

void decompress(unsigned char* buffer, unsigned char* compressed_image, int pitch);
uint8_t getRemaining(unsigned char byteBuffer[][CAMERA_FRAME_BUFFER_SIZE], int section, uint8_t value);
uint16_t getTotalSize(unsigned char byteBuffer[][], int section);
void setRemaining(unsigned char byteBuffer[][], int section, uint8_t value);
//Bytebuffer[size] [0-3] frame index. [4] remaining.
struct BufferItem{
		uint16_t frame_index;
		uint8_t sections_to_be_recieved;
		uint8_t total_number_of_sections;
		struct DataSections* data_sections; //This will contain toatal section # of data
}
struct DataSections{
		uint16_t data_size;
		char* data;
}

