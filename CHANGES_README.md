# RoverSystem
Software Repository for the IEEE Mars Rover project

Changes done to the RoverSystem code: 

1. Created a Logger folder in the src folder with the file logger.cpp and logger.hpp. Also created a Makefile to compile and create a library
2. Created a LibLogger.a so that the logger functions can be used anywhere. 
3. In the rover main.cpp replaced any printfs or couts with the log function. Any code with a exit(1) or return 1 had the Level ERROR and 
any warnings we given the level WARNING, anything else was given the level INFO. 

-Josue Lopez
