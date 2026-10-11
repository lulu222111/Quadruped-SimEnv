cmake_minimum_required(VERSION 3.0.2)
project(competition_perception)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
  tf2_ros
  cv_bridge
  message_filters
  competition_interfaces
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  sensor_msgs
  geometry_msgs
  tf2_ros
  cv_bridge
  message_filters
  competition_interfaces
)

# install(PROGRAMS) preserves the executable bit (install(DIRECTORY)
# does not), so the node stays runnable via rosrun/roslaunch in the
# install space too.  perception_core.py is co-located so the node can
# import it from its own directory.
install(PROGRAMS
  scripts/perception_node
  scripts/perception_core.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# --- Tests ---
# --- 提交包说明（TEST_FREE_SUBMISSION）-------------------------------------
# 测试代码已按要求移出提交包，测试目标声明一并停用，避免 catkin_make 因找不到
# 测试文件而 FATAL_ERROR。此处仅影响测试目标，运行时节点/参数/接口未改动。
# if(CATKIN_ENABLE_TESTING)
#   find_package(catkin REQUIRED COMPONENTS rostest)
#
#   # Pure-Python unit tests (no ROS master needed):
#   #   test_perception.py      — core logic (gate, detection, depth math)
#   #   test_node_wrapper.py    — node wrapper with a fake ROS layer
#   # catkin_add_nosetests takes exactly one test file per invocation.
#   catkin_add_nosetests(test/test_perception.py)
#   catkin_add_nosetests(test/test_node_wrapper.py)
# endif()
