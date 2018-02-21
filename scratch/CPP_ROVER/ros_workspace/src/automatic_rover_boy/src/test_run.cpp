#include <std_msgs/String.h>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include "serial/serial.h"

using namespace std;

int main(int argc, char **argv) {

    /*----SETUP BEGIN----*/
    /*-ROS node setup-*/
    ros::init(argc, argv, "rover_tester"); //init node
    ros::NodeHandle n; //node handler
    ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("test_run", 5); //publisher, max 5 msgs cached at one time
    ros::Rate loop_rate(10); //10Hz every cycle

    /*-Serial setup-*/
    string port = "/dev/ttyACM0"; //Port name
    long baud = 9600; //baudrate
    serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));

    /*-Check serial port-*/
    ROS_INFO_STREAM("Is the serial port open?");
    if(my_serial.isOpen())
        ROS_INFO_STREAM(" Yes. ");
    else
        ROS_INFO_STREAM(" No. ");

    /*-Sample input strings-*/
    string data1 = "0xFC";
    string data2 = "0xFF";
    int count = 0;
    /*----SETUP END----*/

    //1st bit - motor select
    //2nd bit - forwards or backwards
    //3-8 bits - how fast?

    while (ros::ok()) {

        ROS_INFO_STREAM("Iteration: " << count);

        /*
        size_t bytes_wrote = my_serial.write(data1);
        ROS_INFO_STREAM("Bytes written for " << data1 << ": " << bytes_wrote);
        size_t bytes_wrote = my_serial.write(data2);
        ROS_INFO_STREAM("Bytes written for " << data2 << ": " << bytes_wrote);
        */

        count++;

        loop_rate.sleep();
    }

    return 0;
}

