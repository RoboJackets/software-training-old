cmake_minimum_required(VERSION 3.5)
project(localization)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(stsl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)

add_library(${PROJECT_NAME}_component SHARED
  src/motion_model.cpp
  src/aruco_sensor_model.cpp
  src/random_helpers.cpp
  src/particle_filter_localizer.cpp
# BEGIN STUDENT CODE
  src/odometry_sensor_model.cpp
# END STUDENT CODE  
)
ament_target_dependencies(${PROJECT_NAME}_component
  "rclcpp"
  "sensor_msgs"
  "rclcpp_components"
  "visualization_msgs"
  "stsl_interfaces"
  "nav_msgs"
  "tf2_ros"
  "tf2_geometry_msgs"
  "angles"
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

install(
  DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
