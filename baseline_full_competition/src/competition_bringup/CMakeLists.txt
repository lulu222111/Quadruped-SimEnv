cmake_minimum_required(VERSION 3.0.2)
project(competition_bringup)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

# --- 提交包说明（TEST_FREE_SUBMISSION）-------------------------------------
# 测试代码已按要求移出提交包，测试目标声明一并停用，避免构建失败。
# 此处仅影响测试目标；MODE A / MODE B 的 launch 装配与参数透传完全未改动。
# if(CATKIN_ENABLE_TESTING)
#   catkin_add_nosetests(test/test_pipeline_launch.py)
# endif()
