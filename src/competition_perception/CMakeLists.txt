cmake_minimum_required(VERSION 3.0.2)

project(competition_perception)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
  tf2_ros
  cv_bridge
  message_filters
  competition_interfaces
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  sensor_msgs
  geometry_msgs
  tf2_ros
  cv_bridge
  message_filters
  competition_interfaces
)

catkin_install_python(
  PROGRAMS
    scripts/perception_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)