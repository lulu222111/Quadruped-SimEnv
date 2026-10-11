cmake_minimum_required(VERSION 3.0.2)
project(building_generator_classic)

find_package(catkin REQUIRED COMPONENTS
  building_generator_core
  building_generator_interfaces
  gazebo_msgs
  rospy
  tf
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  building_generator_core
  building_generator_interfaces
  gazebo_msgs
  rospy
  tf
)

catkin_install_python(PROGRAMS
  scripts/building_generator_classic_control
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# --- 提交包说明（TEST_FREE_SUBMISSION）-------------------------------------
# 测试代码已按要求移出提交包，测试目标声明一并停用，避免构建失败。
# 此处仅影响测试目标，运行时节点/参数/接口均未改动。
# if(CATKIN_ENABLE_TESTING)
#   catkin_add_nosetests(test)
# endif()
