get_filename_component(EXECUTABLE_NAME "${CMAKE_CURRENT_SOURCE_DIR}" NAME)

file(GLOB SRCS *.cpp)

add_executable(${EXECUTABLE_NAME} ${SRCS})
