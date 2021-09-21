cmake_minimum_required(VERSION 3.5)
project(obstacle_detector)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(OpenCV 4 REQUIRED)

add_library(${PROJECT_NAME} SHARED 
  src/obstacle_detector.cpp
# BEGIN STUDENT CODE
  src/student_functions.cpp
# END STUDENT CODE
)
ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "rclcpp_components"
  "cv_bridge"
  "image_transport"
  "nav_msgs"
  "tf2_ros"
  "tf2_eigen"
  "OpenCV"
)
rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "obstacle_detector::ObstacleDetector"
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
