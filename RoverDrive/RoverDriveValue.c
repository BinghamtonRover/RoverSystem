

#include <stdio.h>
#include<stdlib.h>
#include <stdint.h>
#include <inttypes.h>
#include <math.h>

float forward;

void convert(float inputValue, char * flipped);


int main(){
	system("sudo /sbin/ip link set can0 up type can bitrate 500000");
    //system("cansend can0 007#03");
char* flipped = malloc(sizeof(char) * 100);
  while (1){
	forward=0;
	printf("Enter a number: ");
    	scanf("%f", &forward);
	convert(forward, flipped);
	system(flipped);
	}
free(flipped);
}

void convert(float inputValue, char * flipped){
        char str[8];
        
        float pi = (float)inputValue;
        union { float f; uint32_t u;
                } f2u = { .f = pi };

    sprintf(str, "%x", f2u.u); 
    sprintf(flipped, "cansend can0 00d#%c%c%c%c%c%c%c%c", str[6],str[7],str[4],str[5],str[2],str[3],str[0],str[1]);
	printf("s", flipped);
    //printf("%s", flipped);
}
