cmake_minimum_required(VERSION 3.5)
project(controllers)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(angles REQUIRED)

add_library(controllers SHARED
  # BEGIN STUDENT CODE
  # END STUDENT CODE
  src/lqr_controller.cpp
  src/pid_controller.cpp
  src/test_path_generator.cpp
)
ament_target_dependencies(controllers
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "nav2_core"
  "nav2_msgs"
  "tf2_eigen"
  "tf2_geometry_msgs"
  "Eigen3"
  "angles"
)
set_property(TARGET controllers PROPERTY CXX_STANDARD 17)

# BEGIN STUDENT CODE
# END STUDENT CODE

install(TARGETS controllers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

pluginlib_export_plugin_description_file(nav2_core plugin_descriptions.xml)

ament_package()
