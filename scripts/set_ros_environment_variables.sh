#!/bin/bash
export ROS_MASTER_URI=http://$(cat ~/catkin_ws/src/sensor_dist_ros/scripts/master.cfg):11311
export ROS_IP=$(~/catkin_ws/src/sensor_dist_ros/scripts/getmyip.sh)
export ROS_HOSTNAME=$ROS_IP #`hostname`
#export ROS_NAMESPACE=/`echo $ROS_HOSTNAME | sed -e 's/-/_/g'`;
