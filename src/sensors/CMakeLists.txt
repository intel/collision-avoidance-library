cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

set(SOURCES Sensors.cc)
set(HEADERS Sensors.hh)

if (${WITH_GAZEBO})
    set(SOURCES ${SOURCES} GazeboRealSenseCamera.cc)
    set(HEADERS ${HEADERS} GazeboRealSenseCamera.hh)
endif()

if (${WITH_REALSENSE})
    set(SOURCES ${SOURCES} RealSenseCamera.cc)
    set(HEADERS ${HEADERS} RealSenseCamera.hh)
endif()

export_headers("${HEADERS}" "sensors")
set(COAV_INCLUDE_LIST "${COAV_INCLUDE_LIST}${INCLUDE_LIST}" PARENT_SCOPE)

add_library(sensors OBJECT ${SOURCES})
