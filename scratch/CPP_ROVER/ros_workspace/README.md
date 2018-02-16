# Ok you goddamn scrubs, here's the low down:
## 1. Explanation of file system
    If you didn't ready the tutorials closely enough, or didn't read them at all, here are a few tips:
        * The root folder of the workspace:
            * run 'source devel/setup.bash' so ros knows where your current workspace is
            * _src_ is going to contain any relevant ROS packages we are using
            * run 'catkin_make' to build all packages in src. If it happens to crash, run 'catkin_make -j1'; this will limit the number of parallel jobs to 1 so the Pi won't freak out and run out of swap space. -j4 is the default for 'catkin_make'
## 2. Packages and dependencies
    * All ROS packages, including the one we are building, called 'automatic_rover_boy', should be placed in the <ws>/src folder, where <ws> is the workspace location.
    * If you need to use a package, **MAKE SURE** to also make a reference to it in the CMakeLists file of the 'automatic_rover_boy' package or it won't be recognized, and our package won't be compiled
## 3. test_run.cpp
    This is my current test file. Serial is a package (that can also be used as a regular C++ library!) that we are using to do data transfers. Documentation is sparse ([you can find some here](http://wjwwood.io/serial/doc/1.1.0/index.html)), so make sure to check out the example before looking into the library members. Also, I'm pretty sure I have the bitmapped values wrong, so the rover will be a little funky if you decide to run it.

Note: The ros_workspace folder in the repo in our home directory is **NOT** symbolically linked to the one in our home directory, because Git apparently does not support following them, so changes will be made to only one. Just make sure to commit any changes to the repo folder.

#That is all. Good Luck!
Also, I'm sure I missed a lot of things, so make sure to keep this README.md updated!
