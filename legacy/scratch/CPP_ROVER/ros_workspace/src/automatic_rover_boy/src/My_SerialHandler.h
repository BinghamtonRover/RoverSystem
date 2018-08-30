#ifndef MY_SERIALHANDLER
#define MY_SERIALHANDLER

#include "ros/ros.h"
#include "std_msgs/UInt8.h"
#include "serial/serial.h"

/*
    My_SerialHandler -- class that handles interfacing with serial package and
        serial message callbacks
*/
class My_SerialHandler {
    private:
        serial::Serial *my_serial;
    public:
        My_SerialHandler();
        void serialCallback(const std_msgs::UInt8::ConstPtr& serial_msg);

};
#endif
