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
