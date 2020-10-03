add_executable(testing_node testing_node.cpp)
add_dependencies(testing_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(testing_node ${catkin_LIBRARIES})

install(
        TARGETS testing_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
