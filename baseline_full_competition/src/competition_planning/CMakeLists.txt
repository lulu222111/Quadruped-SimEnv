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

# --- 提交包说明（TEST_FREE_SUBMISSION）-------------------------------------
# 测试代码（含 rostest 用例 planning_node_ros.test）已按要求移出提交包，测试目标
# 声明一并停用，避免 catkin_make 因找不到测试文件而 FATAL_ERROR。
# 此处仅影响测试目标，运行时可执行目标、参数与接口均未改动。
# if(CATKIN_ENABLE_TESTING)
#   find_package(rostest REQUIRED)
#
#   catkin_add_nosetests(test/test_visual_coverage.py)
#   catkin_add_nosetests(test/test_elevator_locator.py)
#   catkin_add_nosetests(test/test_planning_node_generation.py)
#   add_rostest(
#     test/planning_node_ros.test
#   )
# endif()

install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
