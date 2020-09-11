cmake_minimum_required(VERSION 3.17)
project(traffic_light_project)

set(CMAKE_CXX_STANDARD 20)

add_executable(light_controller main.cpp)

add_custom_target(
  copy_resources_to_build
  ALL
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/test_cases/ $<TARGET_FILE_DIR:light_controller>/test_cases
  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/runner.sh $<TARGET_FILE_DIR:light_controller>/runner.sh
  DEPENDS ${CMAKE_SOURCE_DIR}/runner.sh
)
