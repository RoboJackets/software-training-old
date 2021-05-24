cmake_minimum_required(VERSION 3.5)
project(localization)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(stsl_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME}_component SHARED src/particle_filter_localizer.cpp)
ament_target_dependencies(${PROJECT_NAME}_component
  "rclcpp"
  "rclcpp_components"
  "stsl_interfaces"
  "nav_msgs"
  "tf2_ros"
)
rclcpp_components_register_node(
  ${PROJECT_NAME}_component
  PLUGIN "localization::ParticleFilterLocalizer"
  EXECUTABLE ${PROJECT_NAME}_node
)

install(TARGETS ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
