cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set (sources
    GazeboQuadCopter.cc
    MavQuadCopter.cc)

add_library(vehicles ${sources})
target_link_libraries(vehicles
    communication
    mavlink_vehicles
    ${GAZEBO_libraries} ${LDFLAGS})

