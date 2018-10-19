add_executable(inheritance inheritance.cpp RJRobotExtended.h RJRobotExtended.cpp)
add_dependencies(inheritance STSL)
target_link_libraries(inheritance ${STSL_LIBS})
