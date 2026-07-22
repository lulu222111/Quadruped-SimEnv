cmake_minimum_required(VERSION 3.0.2)
project(competition_loop)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
  geometry_msgs
  competition_interfaces
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  std_msgs
  std_srvs
  geometry_msgs
  competition_interfaces
)

# Install Python modules
install(DIRECTORY scripts src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN "__pycache__" EXCLUDE
)
