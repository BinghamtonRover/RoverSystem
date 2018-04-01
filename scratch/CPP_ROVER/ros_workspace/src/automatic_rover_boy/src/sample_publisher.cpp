#include "ros/ros.h"
#include "std_msgs/UInt8.h"

#include <sstream>

int main(int argc, char **argv) {
  ros::init(argc, argv, "sample_serial_sender");

  ros::NodeHandle n;

  ros::Publisher serial_pub = n.advertise<std_msgs::UInt8>("serial", 1000);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok()) {
    std_msgs::UInt8 forward;
    forward.data = 0xFF;

    ROS_INFO("%u count %d", forward, count);
    serial_pub.publish(forward);

    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }


  return 0;
}

