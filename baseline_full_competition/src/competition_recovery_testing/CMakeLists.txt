cmake_minimum_required(VERSION 3.0.2)
project(competition_recovery_testing)

find_package(catkin REQUIRED COMPONENTS
  rospy
  gazebo_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  competition_interfaces
)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS
  rospy
  gazebo_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  competition_interfaces
)

catkin_install_python(PROGRAMS
  scripts/teleport_recovery_coordinator
  scripts/straight_segment_path_source
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# --- 提交包说明（TEST_FREE_SUBMISSION）-------------------------------------
# 测试代码已按要求移出提交包，测试目标声明一并停用，避免构建失败。
# 此处仅影响测试目标，运行时可执行目标、参数与接口均未改动。
# if(CATKIN_ENABLE_TESTING)
#   catkin_add_nosetests(test/test_recovery_lifecycle.py)
#   catkin_add_nosetests(test/test_straight_segment.py)
#   catkin_add_nosetests(test/test_test_only_isolation.py)
# endif()
