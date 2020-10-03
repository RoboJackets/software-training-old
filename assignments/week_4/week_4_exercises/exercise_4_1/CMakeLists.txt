add_executable(your_node your_node.cpp)
add_dependencies(your_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(your_node ${catkin_LIBRARIES})

install(
        TARGETS your_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

