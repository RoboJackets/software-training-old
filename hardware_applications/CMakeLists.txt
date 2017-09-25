cmake_minimum_required(VERSION 3.6.2)
project(hardware_applications)

set(CMAKE_CXX_STANDARD 14)

include_directories(STSL/include)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules)

if(WIN32)
    link_directories(STSL/lib/WINDOWS)
    configure_file(STSL/lib/WINDOWS/libSTSL.dll ${CMAKE_CURRENT_BINARY_DIR}/libSTSL.dll COPYONLY)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
elseif(UNIX AND NOT APPLE)
    link_directories(STSL/lib/LINUX)
    find_package(UDev REQUIRED)
    set(CMAKE_CXX_STANDARD_LIBRARIES ${UDEV_LIBRARIES})
elseif(UNIX AND APPLE)
    link_directories(STSL/lib/OSX)
endif()

add_subdirectory(week1)
add_subdirectory(week2)
