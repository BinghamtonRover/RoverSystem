#include <iostream>
#include <time.h>
#include <fcntl.h>
#include "video_encoder.hpp"
#include "video_system_exception.hpp"

int main() {
	int fd = open("video.h264", O_WRONLY | O_CREAT | O_TRUNC | O_APPEND, 0666);
	if (fd == -1) return -1;
	try {
		VideoEncoder v;
		v.init();
		v.start_video();
		struct timespec spec;
		clock_gettime(CLOCK_MONOTONIC, &spec);
		long now = spec.tv_sec*1000 + spec.tv_nsec / 1.0e6;
		long end = now + 3000;
		
		for (;;) {
			OMX_BUFFERHEADERTYPE *frame = v.get_frame();
			pwrite(fd, frame->pBuffer, frame->nFilledLen, frame->nOffset);
			clock_gettime(CLOCK_MONOTONIC, &spec);
			if (spec.tv_sec*1000 + spec.tv_nsec/1.0e6 >= end) break;
		}
		close(fd);
		return 0;
	} catch (const VideoSystemException& e) {
		std::cout << "Caught an exception: " << e.get_details() << std::endl;
		close(fd);
		return 1;
	}
}
