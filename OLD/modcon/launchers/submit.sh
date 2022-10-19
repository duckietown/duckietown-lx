#!/bin/bash

roscore &
source /environment.sh
source /opt/ros/noetic/setup.bash
source /code/catkin_ws/devel/setup.bash --extend
source /code/submission_ws/devel/setup.bash --extend
source /code/solution/devel/setup.bash --extend

roslaunch --wait agent agent_node.launch &
roslaunch --wait car_interface all.launch veh:=$VEHICLE_NAME &
sleep 5
roslaunch encoder_pose encoder_pose_node.launch veh:=$VEHICLE_NAME AIDO_eval:="true"
# rostopic pub /$VEHICLE_NAME/activity_name std_msgs/String "data: pid_exercise" --latch

