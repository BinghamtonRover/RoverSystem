#include "../../src/network/network.hpp"
#include <linux/joystick.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <fcntl.h> 
#include <unistd.h>

const unsigned short 
    //Button values
    A_BUTTON = 0,
    B_BUTTON = 1,
    X_BUTTON = 2,
    Y_BUTTON = 3,
    LB_BUTTON = 4,
    RB_BUTTON = 5,
    BACK_BUTTON = 6,
    START_BUTTON = 7,
    HOME_BUTTON = 9,
    //Axis values
    JOYSTICK_LEFT_X = 0,
    JOYSTICK_LEFT_Y = 1,
    JOYSTICK_RIGHT_X = 3,
    JOYSTICK_RIGHT_Y = 4,
    DPAD_X = 6,
    DPAD_Y = 7,
    LT_AXIS =2,
    RT_AXIS =5;
//Axis ranges from -32767 to +32767
//If recalibrate is used, the input will be read between 
//POSITIVE: MIN_POS and MAX_POS
//NEGATIVE: -MAX_POS and -MIN_POS
//and then scaled to fit range -32767 to 32767.
//Essentially making MIN_POS and less the new 0 or 'dead' value.
const int16_t 
    MAX_POS = 32767,
    MIN_POS = 5000;
struct movement {
    int16_t left, right;
};

//Joystick values range from -32767 to +32767
//Used formula scaled_val = (new_max - new_min) / (old_max - old_min) * (v - old_min) + new_min
//Can the rover handle the calibration side instead?
int16_t recalibrate(int16_t axis_value){
    int16_t joy_stick_abs = axis_value >=0 ? axis_value : axis_value*-1;
    int16_t new_axis_value = axis_value;
    float percentage = (float(MAX_POS)-0)/(MAX_POS-MIN_POS);
    new_axis_value = percentage*(joy_stick_abs-MIN_POS);
    if (axis_value < 0){
        return -new_axis_value;
     }
        return new_axis_value;
};

int main(int argc, char *argv[]) {
    network::Connection conn;
    network::Error error;
    if ((error = network::connect(&conn, "192.168.1.1", 45545, 45545)) != network::Error::OK) {
        fprintf(stderr, "[!] Failed to connect to rover!\n");
          return 1;
    }
    network::MovementMessage currentMovement = {0};
    network::Buffer* outgoing;
    //Device input defaults to js0
    const char* device_name = "/dev/input/js0";
    if (argc == 2) 
      device_name = argv[1];
    //Controller file descriptor. O_NONBLOCK mdoe ensures multiple events can
    //be read and added to diver queue
    int fd = open(device_name, O_RDONLY | O_NONBLOCK); 
    if (fd == -1){ 
      fprintf(stderr, "[!] Failed to find controller!\n");
      return 1;  
    }
    struct js_event event;
    //Main loop
    while (true) {
        //Loop until event queue is empty, resulting in -1
        while (read(fd, &event, sizeof(event)) > 0) { 
            unsigned short button_number;
            const char* button_state;
            switch (event.type){ 
                //Unused button event handler
  	            case JS_EVENT_BUTTON:
                   button_number = event.number;
                   //Pressed is 1, released is 0
                   button_state =  event.value ? "pressed" : "released"; 

                    break;
                //Only care about the left and right Y values for axis events
                case JS_EVENT_AXIS:
                    switch (event.number) {
                        case JOYSTICK_LEFT_Y:
                            //event.value is defined as a signed 16bit value
                            currentMovement.left = event.value;
                            outgoing = network::get_outgoing_buffer();
                            network::serialize(outgoing, &currentMovement); 
                            network::queue_outgoing(&conn, network::MessageType::MOVEMENT, outgoing);
                            break;
                        case JOYSTICK_RIGHT_Y:
                            currentMovement.right = event.value;
                            outgoing = network::get_outgoing_buffer();
                            network::serialize(outgoing, &currentMovement); 
                            network::queue_outgoing(&conn, network::MessageType::MOVEMENT, outgoing);
                            break;
                    default:
                      //Ignore all other axis events (DPAD and triggers)
                      break;
                    }
                    
                default:
                     //Ignore init events
                     break;
              } 
           
        }  
        //End program if controller is disconnected
        if (errno != EAGAIN) {
            fprintf(stderr, "[!] Controller disconnected\n");
            return 1;
        }
        network::drain_outgoing(&conn);
     }

}

