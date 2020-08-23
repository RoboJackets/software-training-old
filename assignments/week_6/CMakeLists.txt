SubDirList(SUBDIRS ${CMAKE_CURRENT_SOURCE_DIR})

FOREACH(subdir ${SUBDIRS})
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${subdir}/CMakeLists.txt)
        ADD_SUBDIRECTORY(${subdir})
    else()
        message(STATUS "There is no CMakeLists.txt in ${CMAKE_CURRENT_SOURCE_DIR}/${subdir} skipping folder")
    endif()
ENDFOREACH()
