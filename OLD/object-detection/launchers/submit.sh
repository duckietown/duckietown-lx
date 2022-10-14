#!/bin/bash
set -eux

export DEBUG=1
export ROS_HOME=/tmp
export ROSCONSOLE_STDOUT_LINE_BUFFERED=1

source /environment.sh

source /code/catkin_ws/devel/setup.bash --extend
source /code/submission_ws/devel/setup.bash --extend
source /code/solution/devel/setup.bash --extend

roscore &
stdbuf -o L  roslaunch --wait agent agent_node.launch &
stdbuf -o L roslaunch --wait car_interface all.launch veh:=$VEHICLE_NAME &
stdbuf -o L roslaunch --wait object_detection object_detection_node.launch AIDO_eval:="true" veh:=$VEHICLE_NAME


# copy the log files
find /tmp/log  -type f  -name "*.log" -exec cp {} /challenges/challenge-solution-output \;
