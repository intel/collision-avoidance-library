cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set (sources
  GazeboQuadCopter.cc
)

add_library(vehicles ${sources})
target_link_libraries(vehicles communication ${GAZEBO_libraries} ${LDFLAGS})

