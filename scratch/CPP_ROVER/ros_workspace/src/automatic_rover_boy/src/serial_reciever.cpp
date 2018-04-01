#include "ros/ros.h"
#include "std_msgs/UInt8.h"

#include "My_SerialHandler.h"

int main(int argc, char **argv) {

  ros::init(argc, argv, "serial_reciever");
  ros::NodeHandle n;
  My_SerialHandler serial_handler;
  ros::Subscriber sub = n.subscribe("serial", 1000, &My_SerialHandler::serialCallback, &serial_handler);

  ros::spin();

  return 0;
}

