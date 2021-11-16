#!/bin/bash
source /entrypoint.sh
source /opt/ros/noetic/setup.bash
source /code/catkin_ws/devel/setup.bash
rostopic pub /$VEHICLE_NAME/fsm_node/mode duckietown_msgs/FSMState '{header: {}, state: "LANE_FOLLOWING"}'
