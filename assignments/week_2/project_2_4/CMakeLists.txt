cmake_minimum_required(VERSION 3.17)
project(project_2_4)

set(CMAKE_CXX_STANDARD 20)

add_executable(project_2_4 project_2_4.cpp)

add_custom_target(
  copy_resources_to_build
  ALL
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/test_cases/ $<TARGET_FILE_DIR:project_2_4>/test_cases
  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/runner.sh $<TARGET_FILE_DIR:project_2_4>/runner.sh
  DEPENDS ${CMAKE_SOURCE_DIR}/runner.sh
)
