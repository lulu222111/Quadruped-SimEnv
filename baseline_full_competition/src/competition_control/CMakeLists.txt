cmake_minimum_required(VERSION 3.0.2)
project(competition_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  competition_interfaces
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  competition_interfaces
)

# --- 提交包说明（TEST_FREE_SUBMISSION）-------------------------------------
# 原为 install(DIRECTORY scripts test ...)：test/ 已随测试代码移出提交包，
# 故安装规则中的 test 目录一并去除（仅影响测试文件的安装，不影响运行时节点安装）。
install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

# --- 提交包说明（TEST_FREE_SUBMISSION）-------------------------------------
# 电梯执行器相关测试代码已随测试代码整体移出提交包，测试目标声明一并停用，
# 避免 catkin_make 因找不到测试文件而 FATAL_ERROR。
# 执行器本体的运行时源文件（scripts/door_elevator_executor_node 等）与行为未改动。
# if(CATKIN_ENABLE_TESTING)
#   catkin_add_nosetests(test/test_elevator_executor_core_v1.py)
#   catkin_add_nosetests(test/test_elevator_executor_interface_v1.py)
#   catkin_add_nosetests(test/test_elevator_threshold_profile_v1.py)
#   catkin_add_nosetests(test/test_elevator_service_sequence.py)
# endif()
