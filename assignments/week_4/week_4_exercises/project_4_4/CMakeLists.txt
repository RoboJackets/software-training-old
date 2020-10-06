add_executable(pid_node pid_node.cpp)
target_link_libraries(pid_node ${catkin_LIBRARIES})
add_dependencies(pid_node ${catkin_EXPORTED_TARGETS})
