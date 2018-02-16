#include "ros/ros.h"
#include "std_msgs/String.h"


/**
*
* Dummy file for now, but will contain our navigator that will listen
* for navigation instructions on a seperate node
* */

void testCallback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv){
    ros::init(argc, argv, "navigator");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("test_run", 1000, testCallback); //subscribe to "test_run" topic, with testCallback() as the callback function

    ros::spin(); //spin() will not return until this program ends. This will cause it to only care about callback handling

    return 0;
}
