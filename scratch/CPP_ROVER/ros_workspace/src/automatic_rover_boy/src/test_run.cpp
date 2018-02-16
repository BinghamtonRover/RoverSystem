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

    ros::init(argc, argv, "rover_tester");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("test_run", 5);
    ros::Rate loop_rate(10);

    string port = "/dev/ttyACM0"; //Port name
    long baud = 9600; //baudrate
    serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));

    ROS_INFO_STREAM("Is the serial port open?");
    if(my_serial.isOpen())
        ROS_INFO_STREAM(" Yes. ");
    else
        ROS_INFO_STREAM(" No. ");

    string data1 = "0xFC";
    string data2 = "0xFF";
    int count = 0;

    while (ros::ok()) {
        /*
        geometry_msgs::Twist msg;
        msg.linear.x = msg.linear.y = msg.linear.z = 0;
        msg.angular.x = msg.angular.y = msg.angular.z = 0;

        chatter_pub.publish(msg);
        */
        ROS_INFO_STREAM("Iteration: " << count);

        size_t bytes_wrote = my_serial.write(data1);
        ROS_INFO_STREAM("Bytes written for data1: " << bytes_wrote);
        bytes_wrote = my_serial.write(data2);
        ROS_INFO_STREAM("Bytes written for data2: " << bytes_wrote);

        count++;

        loop_rate.sleep();
    }

    return 0;
}

