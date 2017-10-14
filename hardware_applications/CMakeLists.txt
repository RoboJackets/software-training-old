cmake_minimum_required(VERSION 3.5.1)
project(hardware_applications)

set(CMAKE_CXX_STANDARD 14)

include(ExternalProject)

ExternalProject_Add(STSL
        GIT_REPOSITORY https://github.com/RoboJackets/stsl.git
        INSTALL_DIR ${PROJECT_BINARY_DIR}
        CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>)

set(STSL_LIBS ${PROJECT_BINARY_DIR}/lib/libSTSL.a)

include_directories(${PROJECT_BINARY_DIR}/include)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules)

if(UNIX AND NOT APPLE)
    find_package(UDev REQUIRED)
    set(CMAKE_CXX_STANDARD_LIBRARIES ${UDEV_LIBRARIES})
endif()

add_subdirectory(week1)
add_subdirectory(week2)
add_subdirectory(week3)
