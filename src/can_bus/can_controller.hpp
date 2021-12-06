#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "lib.h"

#include <inttypes.h>
#include <stdlib.h>

#define num_devices 3

int can_send_velocity(int device_num, char* message);
int can_send_velocity(int device_num, uint32_t message);
int can_send_velocity(int device_num, float message);
int can_send_custom_message(char* custom_message);

char* get_can_velocity_message(int device_num, uint32_t message);
uint32_t get_big_endian(uint32_t u);
int can_send(char* argv1, char* argv2);