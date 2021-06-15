#!/bin/bash

export ROSCONSOLE_STDOUT_LINE_BUFFERED=1

pip3 install git+https://github.com/Velythyl/dataclasses-spoof
pip3 install git+https://github.com/duckietown/lib-dt-mooc-2021

source /environment.sh

source /code/catkin_ws/devel/setup.bash --extend
source /code/submission_ws/devel/setup.bash --extend
source /code/solution/devel/setup.bash --extend

roslaunch --wait agent agent_node.launch &
roslaunch --wait car_interface all.launch veh:=$VEHICLE_NAME &
roslaunch --wait object_detection object_detection_node.launch veh:=$VEHICLE_NAME

