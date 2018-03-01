To run the tennis ball code ensure ros and opencv are installed.

**
FYI: Be sure to remove the build directory in ros_workspace before you catkin_make or it won't work.
rm -r build
**


In the scratch/CPP_ROVER/ros_workspace directory run the following terminal commands:

source /opt/ros/kinetic/setup.bash
source devel/setup.bash
catkin_make
rosrun BallTracking NallTracking_node


---
This should run Josiahs code, currently it prints the distance of a tennis ball.
The source commands can be placed in .bashrc


