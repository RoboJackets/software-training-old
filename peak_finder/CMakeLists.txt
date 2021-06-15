cmake_minimum_required(VERSION 3.5)
project(peak_finder)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(stsl_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(nav2_msgs REQUIRED)

add_library(${PROJECT_NAME}_components SHARED src/peak_finder_component.cpp)
ament_target_dependencies(${PROJECT_NAME}_components
  "rclcpp"
  "rclcpp_components"
  "rclcpp_action"
  "stsl_interfaces"
  "tf2_ros"
  "tf2_eigen"
  "nav2_msgs"
)
rclcpp_components_register_node(
  ${PROJECT_NAME}_components
  PLUGIN "peak_finder::PeakFinderComponent"
  EXECUTABLE ${PROJECT_NAME}_node
)

install(TARGETS ${PROJECT_NAME}_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
