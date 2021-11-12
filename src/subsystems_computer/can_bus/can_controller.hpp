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

void can_init_drive();
//will add something like this if needed
//int can_drive(uint32_t power_1, uint32_t power_2, uint32_t power_3, uint32_t power_4, uint32_t power_5, uint32_t power_6);

int can_init(int device_num);
int can_send(int device_num, char* message);
int can_send(int device_num, uint32_t message);
int can_send(int device_num, float message);
int can_send_custom_message(int device_num, char* custom_message);

char* get_can_device(int device_num);
char* get_can_message(int device_num, uint32_t message);
uint32_t get_big_endian(uint32_t u);
int can_send(char* argv1, char* argv2);