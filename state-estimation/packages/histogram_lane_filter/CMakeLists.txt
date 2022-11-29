cmake_minimum_required(VERSION 2.8.3)
project(histogram_lane_filter)

find_package(catkin REQUIRED COMPONENTS
  rospy
  duckietown_msgs # Every duckietown packages should use this.
  cv_bridge
  tf
)


catkin_package()


include_directories(
  ${catkin_INCLUDE_DIRS}
)
