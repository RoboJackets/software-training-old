cmake_minimum_required(VERSION 3.0.2)
project(week_6_exercises)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
             roscpp
             actionlib
             actionlib_msgs
             turtle_actionlib
             std_msgs
             geometry_msgs
             nav_msgs
             cv_bridge
             image_transport
             message_generation
        )

find_package(OpenCV REQUIRED)

catkin_package(
        CATKIN_DEPENDS roscpp
)

include_directories(${catkin_INCLUDE_DIRS})
add_subdirectory(project_6)
add_subdirectory(exercise_6_1)
add_subdirectory(exercise_6_2)
