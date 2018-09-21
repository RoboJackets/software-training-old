add_executable(week5 starter_main.cpp)
add_dependencies(week5 STSL)
target_link_libraries(week5 ${STSL_LIBS})

add_executable(week5_final final_main.cpp)
add_dependencies(week5_final STSL)
target_link_libraries(week5_final ${STSL_LIBS})
