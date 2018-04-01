#include "My_SerialHandler.h"

My_SerialHandler::My_SerialHandler(){
    //initialize my_serial
    this->my_serial = new serial::Serial("/dev/ttyACM0", 9600, serial::Timeout::simpleTimeout(1000));
    this->my_serial->setTimeout(serial::Timeout::max(), 250, 0, 250, 0);
};

void My_SerialHandler::serialCallback(const std_msgs::UInt8::ConstPtr& serial_msg) {
    unsigned char msg[2];
    msg[0] = serial_msg->data; msg[1] = 0;

    ROS_INFO("Recieved %c", msg[0]);
    this->my_serial->write(msg, 1);
};
