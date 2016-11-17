cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(SOURCES MavQuadCopter.cc)
set(LIBRARIES mavlink_vehicles)

if (${WITH_GAZEBO})
    set(SOURCES ${SOURCES} GazeboQuadCopter.cc)
    set(LIBRARIES ${LIBRARIES} communication ${GAZEBO_libraries})
endif()

add_library(vehicles ${SOURCES})
target_link_libraries(vehicles ${LIBRARIES})
