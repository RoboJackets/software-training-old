add_executable(color_binning color_binning.cpp)
add_dependencies(color_binning STSL)
target_link_libraries(color_binning ${STSL_LIBS})
