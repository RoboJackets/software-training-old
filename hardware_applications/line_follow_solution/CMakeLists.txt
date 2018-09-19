add_executable(line_follow_solution main.cpp)
add_dependencies(line_follow_solution STSL)
target_link_libraries(line_follow_solution ${STSL_LIBS})
