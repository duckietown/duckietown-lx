#!/bin/bash

source /environment.sh
source /opt/ros/noetic/setup.bash
source /code/catkin_ws/devel/setup.bash
source /code/exercise_ws/devel/setup.bash
roslaunch --wait duckietown_demos lane_following.launch