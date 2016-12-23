#!/bin/bash
rosparam set use_sim_time true
export ROS_NAMESPACE=RosAria

rosrun gmapping slam_gmapping scan:=laser odom_frame:=base_link
