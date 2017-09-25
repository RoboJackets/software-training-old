add_executable(week2 starter_main.cpp starter_helpers.cpp starter_helpers.h)
add_dependencies(week2 STSL)
target_link_libraries(week2 ${STSL_LIBS})

add_executable(week2_final final_main.cpp final_helpers.cpp final_helpers.h)
add_dependencies(week2_final STSL)
target_link_libraries(week2_final ${STSL_LIBS})
