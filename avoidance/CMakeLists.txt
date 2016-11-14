cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

set (sources
  QuadCopterShiftAvoidance.cc
  QuadCopterVFFAvoidance.cc
  QuadCopterStopAvoidance.cc
)

add_library(avoidance ${sources})
target_link_libraries(avoidance
  common
  vehicles
  ${GAZEBO_libraries}
  ${LDFLAGS}
)

