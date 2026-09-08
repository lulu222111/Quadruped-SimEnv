cmake_minimum_required(VERSION 3.0.2)
project(competition_loop)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
  competition_interfaces
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
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

# --- Tests ---
if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest)

  # Pure-Python unit tests (no ROS master needed)
  catkin_add_nosetests(test/test_interface_validator.py)
  catkin_add_nosetests(test/test_time_keeper.py)
  catkin_add_nosetests(test/test_state_machine.py)
  catkin_add_nosetests(test/test_result_writer.py)
  catkin_add_nosetests(test/test_hazard_candidate_router.py)
  catkin_add_nosetests(test/test_competition_loop_node.py)
  catkin_add_nosetests(test/test_returning_launch.py)
  catkin_add_nosetests(test/test_elevator_interface_freeze.py)
  catkin_add_nosetests(test/test_floor_lifecycle.py)
endif()
