cmake_minimum_required(VERSION 3.6.2)
project(hardware_applications)

set(CMAKE_CXX_STANDARD 14)

include_directories(STSL/include)

if(WIN32)
    link_directories(STSL/lib/WINDOWS)
    configure_file(STSL/lib/WINDOWS/libSTSL.dll ${CMAKE_CURRENT_BINARY_DIR}/libSTSL.dll COPYONLY)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
elseif(UNIX AND NOT APPLE)
    link_directories(STSL/lib/LINUX)
    set(CMAKE_CXX_STANDARD_LIBRARIES /usr/lib/x86_64-linux-gnu/libudev.so)
elseif(UNIX AND APPLE)
    link_directories(STSL/lib/OSX)
endif()

add_subdirectory(week1)
