cmake_minimum_required(VERSION 3.0.2)
project(competition_mapping)

find_package(catkin REQUIRED COMPONENTS
  competition_interfaces
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  tf2_ros
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    competition_interfaces
    geometry_msgs
    nav_msgs
    rospy
    sensor_msgs
    std_msgs
    tf2_ros
)

catkin_install_python(PROGRAMS
  scripts/mapping_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/test_occupancy_grid_core.py)
  add_rostest(test/mapping_integration.test)
endif()
