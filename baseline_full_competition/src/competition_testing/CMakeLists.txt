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
# --- 提交包说明（TEST_FREE_SUBMISSION）-------------------------------------
# 测试代码已按要求移出提交包，测试目标声明一并停用，避免构建失败。
# 此处仅影响测试目标，运行时可执行目标、参数与接口均未改动。
# if(CATKIN_ENABLE_TESTING)
#   # Pure-Python unit test (no ROS master needed)
#   catkin_add_nosetests(test/test_mock_isolation.py)
# endif()
