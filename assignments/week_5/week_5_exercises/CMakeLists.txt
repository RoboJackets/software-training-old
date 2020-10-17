cmake_minimum_required(VERSION 3.0.2)
project(week_5_exercises)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
        roscpp
        buzzsim
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES week_5_exercises
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
${catkin_INCLUDE_DIRS}
)

add_executable(printing_node exercise_5_1/printing_node.cpp)
add_dependencies(printing_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(printing_node ${catkin_LIBRARIES})

add_subdirectory(project_5_4)
add_subdirectory(project_5_5)

install(
        TARGETS printing_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
