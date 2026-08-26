cmake_minimum_required(VERSION 3.0.2)
project(competition_planning)

find_package(catkin REQUIRED COMPONENTS
  rospy
  nav_msgs
  geometry_msgs
  std_msgs
  competition_interfaces
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  nav_msgs
  geometry_msgs
  std_msgs
  competition_interfaces
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  catkin_add_nosetests(test/test_visual_coverage.py)
  add_rostest(
    test/planning_node_ros.test
  )
endif()

install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
