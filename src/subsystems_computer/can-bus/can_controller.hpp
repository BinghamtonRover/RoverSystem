#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "lib-char.h"

#include <inttypes.h>
#include <queue>

class can_controller {
    public:
        can_controller(int throttle, int num_cans, int q_lim);
        int can_update(int delta_time);
        int can_send(int frame_num, char* prefix, char* message);
        int can_send(int frame_num, char* prefix, uint32_t message);
        int can_send_to_queue(int frame_num, char* prefix, char* message);
        int can_send_to_queue(int frame_num, char* prefix, uint32_t message);
    private:
        uint32_t get_big_endian(uint32_t u);
        int can_send(int argv1, char* argv2);
        int can_send(char* argv1, char* argv2);
        int* throttle_timers;
        int* throttle_times;
        queue<char*>* message_queue;
        int can_frame_count;
        int max_queue_size;
};