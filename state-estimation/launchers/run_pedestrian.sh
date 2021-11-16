#!/bin/bash

source /environment.sh
source /code/catkin_ws/devel/setup.bash --extend
source /code/submission_ws/devel/setup.bash --extend
source /code/solution/devel/setup.bash --extend

git clone https://github.com/Velythyl/dataclasses-spoof.git dataclasses && cd dataclasses &&  pip3 install . && cd ..
pip3 install git+https://github.com/duckietown/lib-dt-mooc-2021

# initialize launch file
dt-launchfile-init

# launching app

dt-exec roslaunch --wait agent agent_node.launch
dt-exec roslaunch --wait car_interface all.launch veh:=$VEHICLE_NAME
dt-exec roslaunch --wait duckietown_demos lane_following_pedestrians.launch


# ----------------------------------------------------------------------------

# wait for app to end
dt-launchfile-join


