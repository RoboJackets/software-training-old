cmake_minimum_required(VERSION 3.5)
project(mineral_deposit_tracking)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(stsl_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/kalman_filter.cpp
  src/mineral_deposit_tracker.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  stsl_interfaces
  geometry_msgs
  Eigen3
  tf2_ros
  tf2_eigen
)
rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "mineral_deposit_tracking::MineralDepositTracker"
  EXECUTABLE mineral_deposit_tracker
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
