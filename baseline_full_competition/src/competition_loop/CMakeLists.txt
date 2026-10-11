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
# --- 提交包说明（TEST_FREE_SUBMISSION）-------------------------------------
# 测试代码已按要求移出提交包，测试目标声明一并停用，避免 catkin_make 因找不到
# 测试文件而 FATAL_ERROR。此处仅影响测试目标，运行时目标/参数/接口未改动。
# if(CATKIN_ENABLE_TESTING)
#   find_package(catkin REQUIRED COMPONENTS rostest)
#
#   # Pure-Python unit tests (no ROS master needed)
#   catkin_add_nosetests(test/test_interface_validator.py)
#   catkin_add_nosetests(test/test_time_keeper.py)
#   catkin_add_nosetests(test/test_state_machine.py)
#   catkin_add_nosetests(test/test_result_writer.py)
#   catkin_add_nosetests(test/test_hazard_candidate_router.py)
#   catkin_add_nosetests(test/test_competition_loop_node.py)
#   catkin_add_nosetests(test/test_returning_launch.py)
#   catkin_add_nosetests(test/test_elevator_interface_freeze.py)
#   catkin_add_nosetests(test/test_mission_plan.py)
#   catkin_add_nosetests(test/test_floor_mission_fsm.py)
# endif()
