cmake_minimum_required(VERSION 3.0.2)
project(competition_localization)

find_package(catkin REQUIRED COMPONENTS
  rospy
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  competition_interfaces
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  competition_interfaces
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()

catkin_install_python(PROGRAMS
  scripts/localization_node
  scripts/leg_odometry.py
  scripts/filter_livox.py
  scripts/mock_fast_lio_odom.py
  scripts/verify_localization_consistency.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
