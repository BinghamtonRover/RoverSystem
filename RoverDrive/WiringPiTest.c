#include <wiringPiSPI.h>
#include <stdio.h>

int main(){
	char buffer[32];
	int setup = wiringPiSPISetup(1, 500000);
	while(1){

		int read = wiringPiSPIDataRW(1, buffer, 32);
		printf("%s", buffer);

	}


}
