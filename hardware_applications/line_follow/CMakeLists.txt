add_executable(line_follow line_follow.cpp)
add_dependencies(line_follow STSL)
target_link_libraries(line_follow ${STSL_LIBS})
