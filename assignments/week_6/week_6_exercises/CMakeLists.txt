cmake_minimum_required(VERSION 3.0.2)
project(week_6_exercises)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
             roscpp
             std_msgs
             geometry_msgs
             nav_msgs
             cv_bridge
             image_transport
        )

find_package(OpenCV REQUIRED)

catkin_package(
        CATKIN_DEPENDS roscpp
)

include_directories(${catkin_INCLUDE_DIRS})
add_subdirectory(project_6)
