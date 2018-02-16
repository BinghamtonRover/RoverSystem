#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_run"); //specify argc and argv, and our unique node name(test_run)
    ros::NodeHandle n; //handler to help initialize node for us
    ros::Publisher output_pub = n.advertise<std_msgs::String>("test", 1000); //create Publisher object that publishes std_msgs::String objects over the "test" topic at max 1000 char limit
    ros::Rate loop_rate(10); //loop at 10Hz (10 times a second)
    int count = 0;
    while(ros::ok()) { //ros::ok() lets us know if the signal handler has recieved a signal to kill the program. Return of true means its okay to go :)
        std_msgs::String msg;
        std::stringstream ss;
        ss << "test " << count;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str()); //ROS_INFO is replacement for cout

        output_pub.publish(msg); //finally, publish msg to topic

        /*http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning*/
        ros::spinOnce(); //visit above link for more info, but essentially, if this node were subscribed to a topic (it isn't now), it would be able to process the subscriber callback during *this* time frame. This lets you run callback code while having your own program be running as well
        loop_rate.sleep(); //sleep until our specified loop_rate is reached
        count++;
    }

    return 0;
}
