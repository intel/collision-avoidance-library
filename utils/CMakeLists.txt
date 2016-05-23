cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set (sources
  PPTree.cc
)

add_library(utils ${sources})
target_link_libraries(utils ${GAZEBO_libraries} ${LDFLAGS})

