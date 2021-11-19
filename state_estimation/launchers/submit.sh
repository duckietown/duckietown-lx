#!/bin/bash

source /environment.sh

source /opt/ros/noetic/setup.bash
source /code/catkin_ws/devel/setup.bash --extend
source /code/submission_ws/devel/setup.bash --extend
source /code/solution/devel/setup.bash --extend

set -eux

dt-exec-BG roscore
dt-exec-BG roslaunch --wait car_interface all.launch veh:="${VEHICLE_NAME}"
dt-exec-BG roslaunch --wait duckietown_demos lane_following.launch

sleep 5
dt-exec-BG roslaunch --wait duckietown_demos set_state.launch veh:="${VEHICLE_NAME}" state:="LANE_FOLLOWING"


dt-exec-FG roslaunch --wait agent agent_node.launch || true
rostopic list
copy-ros-logs