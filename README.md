# WIT IMU DRIVER for ROS2 

* wit_imu_driver_node.cpp - Contains ROS2 publishers, subscribers and service to interface with WIT IMU libraries
* wt901.cpp - Contains WIT IMU library code

## Issue while launching

* The start_read() function in the wit_imu_driver_node.cpp file at line 274, calls the pushBytes() function in wt901.cpp file, but the buf_[0] != 0x55 and this causes, the function to start erasing the buffer values that were read. Seems to be caused as a result of wrong data read in from the start_read() function call.

