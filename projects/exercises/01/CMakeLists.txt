add_executable(drive_in_line drive_in_line.cpp)
add_dependencies(drive_in_line STSL)
target_link_libraries(drive_in_line ${STSL_LIBS})
