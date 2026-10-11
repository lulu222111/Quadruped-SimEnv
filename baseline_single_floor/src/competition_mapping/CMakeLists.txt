cmake_minimum_required(VERSION 3.0.2)
project(competition_mapping)

find_package(catkin REQUIRED COMPONENTS
  competition_interfaces
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  tf2_ros
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    competition_interfaces
    geometry_msgs
    nav_msgs
    rospy
    sensor_msgs
    std_msgs
    tf2_ros
)

catkin_install_python(PROGRAMS
  scripts/mapping_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# --- 提交包说明（TEST_FREE_SUBMISSION）-------------------------------------
# 测试代码（test/ 目录与 *.test rostest 文件）已按要求移出提交包，测试目标声明
# 一并停用，避免 catkin_make 因找不到测试文件而 FATAL_ERROR。
# 此处仅影响测试目标，运行时可执行目标、参数与接口均未改动。
# if(CATKIN_ENABLE_TESTING)
#   find_package(rostest REQUIRED)
#   catkin_add_nosetests(test/test_occupancy_grid_core.py)
#   catkin_add_nosetests(test/test_mapping_node_safety.py)
#   add_rostest(test/mapping_integration.test)
#   add_rostest(test/mapping_faults.test)
#   add_rostest(test/mapping_clock_rollback.test)
# endif()