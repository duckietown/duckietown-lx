#!/bin/bash
sed -i "s/agent/${VEHICLE_NAME}/g" /opt/ros/noetic/share/rviz/odometry.rviz
rviz -d /opt/ros/noetic/share/rviz/odometry.rviz &
python3 /root/Documents/PID-control-tool.py &
rostopic pub /$VEHICLE_NAME/activity_name std_msgs/String "data: pid"
#rostopic pub /$VEHICLE_NAME/joy_mapper_node/car_cmd duckietown_msgs/Twist2DStamped "{header: auto,  v: 0, omega: 0}"
