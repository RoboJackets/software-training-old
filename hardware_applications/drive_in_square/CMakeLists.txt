add_executable(drive_in_square drive_in_square.cpp)
add_dependencies(drive_in_square STSL)
target_link_libraries(drive_in_square ${STSL_LIBS})
