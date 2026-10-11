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

# --- 提交包说明（TEST_FREE_SUBMISSION）-------------------------------------
# 本提交包按比赛要求不包含测试代码，test/ 目录已整体移除；对应的测试目标声明
# 一并停用，否则 catkin_make 在 CATKIN_ENABLE_TESTING=ON 时会因找不到测试文件而
# 直接 FATAL_ERROR。此处仅影响测试目标，运行时可执行目标、参数与接口均未改动。
# if(CATKIN_ENABLE_TESTING)
#   catkin_add_nosetests(test)
# endif()

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
