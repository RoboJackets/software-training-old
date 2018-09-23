add_executable(median_line median_line.cpp)
add_dependencies(median_line STSL)
target_link_libraries(median_line ${STSL_LIBS})
