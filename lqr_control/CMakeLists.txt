cmake_minimum_required(VERSION 3.5)
project(control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(lqr_control SHARED
  src/controller_test_client.cpp
  src/lqr_controller.cpp
  src/test_path_generator.cpp
)
ament_target_dependencies(lqr_control
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "nav2_core"
  "nav2_msgs"
  "tf2_eigen"
  "tf2_geometry_msgs"
  "Eigen3"
)
set_property(TARGET lqr_control PROPERTY CXX_STANDARD 17)

rclcpp_components_register_node(
  lqr_control
  PLUGIN "lqr_control::ControllerTestClient"
  EXECUTABLE lqr_controller_test_client
)

install(TARGETS lqr_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_library(mpc_control SHARED
        src/controller_test_client.cpp
        src/mpc_controller.cpp
        src/test_path_generator.cpp
        )
ament_target_dependencies(mpc_control
        "rclcpp"
        "rclcpp_action"
        "rclcpp_components"
        "nav2_core"
        "nav2_msgs"
        "tf2_eigen"
        "tf2_geometry_msgs"
        "Eigen3"
        )
set_property(TARGET mpc_control PROPERTY CXX_STANDARD 17)

rclcpp_components_register_node(
        mpc_control
        PLUGIN "mpc_control::ControllerTestClient"
        EXECUTABLE mpc_controller_test_client
)

install(TARGETS mpc_control
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
