cmake_minimum_required(VERSION 3.5)
project(rj_training_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
  behavior_trees
  config
  launch
  rviz_config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
