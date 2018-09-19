add_executable(median_line_solution main.cpp)
add_dependencies(median_line_solution STSL)
target_link_libraries(median_line_solution ${STSL_LIBS})
