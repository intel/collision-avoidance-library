cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

if (${WITH_GAZEBO})
    set(SOURCES ${SOURCES} GazeboRealSenseCamera.cc)
    set(LIBRARIES ${LIBRARIES} communication ${GAZEBO_LIBRARIES})
endif()

if (${WITH_REALSENSE})
    set(SOURCES ${SOURCES} RealSenseCamera.cc)
    set(LIBRARIES ${LIBRARIES} ${REALSENSE_LIBRARIES})
endif()

if (DEFINED SOURCES AND DEFINED LIBRARIES)
    add_library(sensors ${SOURCES})
    target_link_libraries(sensors ${LIBRARIES})
endif()

