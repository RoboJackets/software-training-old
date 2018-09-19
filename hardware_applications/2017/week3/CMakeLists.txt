add_executable(week3 starter_main.cpp)
add_dependencies(week3 STSL)
target_link_libraries(week3 ${STSL_LIBS})

add_executable(week3_final final_main.cpp)
add_dependencies(week3_final STSL)
target_link_libraries(week3_final ${STSL_LIBS})
