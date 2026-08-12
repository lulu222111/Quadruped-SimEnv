cmake_minimum_required(VERSION 3.0.2)
project(competition_testing)

find_package(catkin REQUIRED COMPONENTS
  rospy
  nav_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  competition_interfaces
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  nav_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  competition_interfaces
)

install(DIRECTORY scripts launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

# --- Tests ---
if(CATKIN_ENABLE_TESTING)
  # Pure-Python unit test (no ROS master needed)
  catkin_add_nosetests(test/test_mock_isolation.py)
endif()
