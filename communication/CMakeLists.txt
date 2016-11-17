cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

if (${WITH_GAZEBO})
    set(SOURCES ${SOURCES} GazeboContext.cc)
    set(LIBRARIES ${LIBRARIES} ${GAZEBO_LIBRARIES})
endif()

if (DEFINED SOURCES AND DEFINED LIBRARIES)
    add_library(communication ${SOURCES})
    target_link_libraries(communication ${LIBRARIES})
endif()

