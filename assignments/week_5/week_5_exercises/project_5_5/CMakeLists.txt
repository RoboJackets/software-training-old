add_executable(buzzsim_helper buzzsim_helper.cpp)
target_link_libraries(buzzsim_helper ${catkin_LIBRARIES})

add_executable(kalman_filter_node kalman_filter.cpp)
target_link_libraries(kalman_filter_node ${catkin_LIBRARIES})

install(
        TARGETS buzzsim_helper kalman_filter_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

