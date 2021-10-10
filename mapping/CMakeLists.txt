cmake_minimum_required(VERSION 3.5)
project(mapping)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/mapping_node.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "rclcpp_components"
  "nav_msgs"
  "tf2"
)
rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "mapping::MappingNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

install(TARGETS ${PROJECT_NAME}
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
