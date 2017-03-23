cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

if (${WITH_GAZEBO})
    set(SOURCES ${SOURCES} GazeboRealSenseCamera.cc)
endif()

if (${WITH_REALSENSE})
    set(SOURCES ${SOURCES} RealSenseCamera.cc)
endif()

add_library(sensors OBJECT ${SOURCES})
