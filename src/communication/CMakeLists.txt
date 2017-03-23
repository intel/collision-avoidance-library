cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

if (${WITH_GAZEBO})
    set(SOURCES ${SOURCES} GazeboContext.cc)
endif()

add_library(communication OBJECT ${SOURCES})
