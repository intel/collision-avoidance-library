cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set (sources
  QuadCopterVFFAvoidance.cc
)

add_library(avoidance ${sources})
target_link_libraries(avoidance common ${GAZEBO_libraries} ${LDFLAGS})

