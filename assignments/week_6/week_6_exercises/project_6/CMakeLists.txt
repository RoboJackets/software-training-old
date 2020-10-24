add_executable(start_light start_light.cpp)
target_include_directories(start_light PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(start_light ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(start_light ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
