cmake_minimum_required(VERSION 2.8)
project(week6)
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 14)

add_executable(main main.cpp)
target_link_libraries(main ${OpenCV_LIBS})
