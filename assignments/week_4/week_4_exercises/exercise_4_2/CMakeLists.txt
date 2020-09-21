add_executable(rqt_plot_node exercise_4_2.cpp)
target_link_libraries(rqt_plot_node ${catkin_LIBRARIES} ${OpenCV_INCLUDE_DIRS})
add_definitions(-DCUR_DIR=${CMAKE_CURRENT_SOURCE_DIR})
