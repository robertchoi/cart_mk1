#!/bin/bash
source /home/cart/.bashrc
source /opt/ros/melodic/setup.bash
source /home/cart/catkin_ws/devel/setup.bash

roslaunch ros_servo run_all_temp_v1.launch
