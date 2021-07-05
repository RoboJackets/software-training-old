cmake_minimum_required(VERSION 3.5)
project(coordinate_transform)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(stsl_interfaces REQUIRED)

add_library(${PROJECT_NAME}_components SHARED src/coordinate_transform_component.cpp)
ament_target_dependencies(${PROJECT_NAME}_components
        "rclcpp"
        "rclcpp_components"
        "tf2_ros"
        "tf2_eigen"
        "visualization_msgs"
        "stsl_interfaces"
        )
rclcpp_components_register_node(
        ${PROJECT_NAME}_components
        PLUGIN "coordinate_transform::CoordinateTransformComponent"
        EXECUTABLE ${PROJECT_NAME}_node
)

install(TARGETS ${PROJECT_NAME}_components
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
