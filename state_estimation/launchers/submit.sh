#!/bin/bash
roscore &
source /environment.sh
source /opt/ros/noetic/setup.bash
source /code/catkin_ws/devel/setup.bash
source /code/exercise_ws/devel/setup.bash
python3 /code/solution.py &
roslaunch --wait car_interface all.launch veh:=$VEHICLE_NAME &
roslaunch --wait duckietown_demos lane_following.launch &
sleep 5
rostopic pub /$VEHICLE_NAME/fsm_node/mode duckietown_msgs/FSMState '{header: {}, state: "LANE_FOLLOWING"}'
