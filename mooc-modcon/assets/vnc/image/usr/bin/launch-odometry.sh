#!/bin/bash
sed -i "s/agent/${VEHICLE_NAME}/g" /opt/ros/noetic/share/rviz/odometry.rviz
rostopic pub /$VEHICLE_NAME/activity_name std_msgs/String "data: odometry" &
dt-launcher-joystick & 
rviz -d /opt/ros/noetic/share/rviz/odometry.rviz