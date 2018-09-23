add_executable(system_test system_test.cpp)
add_dependencies(system_test STSL)
target_link_libraries(system_test ${STSL_LIBS})
