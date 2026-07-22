cmake_minimum_required(VERSION 3.0.2)
project(competition_localization)

find_package(catkin REQUIRED COMPONENTS
  rospy
  nav_msgs
  sensor_msgs
  tf2_ros
  competition_interfaces
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  nav_msgs
  sensor_msgs
  tf2_ros
  competition_interfaces
)

install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
