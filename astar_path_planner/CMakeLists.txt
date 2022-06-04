cmake_minimum_required(VERSION 3.5)
project(astar_path_planner)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_core REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)

set(dependencies
  rclcpp
  nav2_core
  angles
  tf2_eigen
  Eigen3
)

add_library(${PROJECT_NAME} SHARED
  src/astar_path_planner_plugin.cpp
  src/astar_path_planner.cpp
  src/utils.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(nav2_core astar_path_planner_libraries.xml)

ament_package()
