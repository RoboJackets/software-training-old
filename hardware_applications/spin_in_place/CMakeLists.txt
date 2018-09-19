add_executable(spin_in_place main.cpp)
add_dependencies(spin_in_place STSL)
target_link_libraries(spin_in_place ${STSL_LIBS})
