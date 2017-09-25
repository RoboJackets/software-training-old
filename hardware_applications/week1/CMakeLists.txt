add_executable(week1_final final_code.cpp)
add_dependencies(week1_final STSL)
target_link_libraries(week1_final ${STSL_LIBS})
