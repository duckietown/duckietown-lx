#!/bin/bash
source /entrypoint.sh
source /opt/ros/noetic/setup.bash
source /code/catkin_ws/devel/setup.bash
rostopic pub /$VEHICLE_NAME/fsm_node/mode duckietown_msgs/FSMState '{header: {}, state: "NORMAL_JOYSTICK_CONTROL"}' -1 &
rostopic pub /$VEHICLE_NAME/joy_mapper_node/car_cmd duckietown_msgs/Twist2DStamped '{header: {}, v: 0.0, omega: 0.0}' -1

