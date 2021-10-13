#include <stdio.h>
#include<stdlib.h>
#include <stdint.h>
#include <inttypes.h>
#include <math.h>

char forward;




int main(){
    //system("cansend can0 007#03");
    
  while (1){ forward=0;
    //printf("\n Enter a Forward Value: ");
    scanf("%c", &forward );
    //forward = getchar();
    //printf("\n you entered: %d", forward);
    
    
    if (forward == 'w'){
        //printf("Forward");
        system("cansend can0 00d#00.00.80.3f");
    }else if (forward == 's'){
        //printf("Backwards");
        system("cansend can0 00d#00.00.80.bf");
    }else{
       // system("cansend can0 00d#00.00.00.00.00");
    }
    //forward1 = forward; system('cansend can0 f);
  }
  return 0;
}

