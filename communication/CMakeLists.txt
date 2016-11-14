cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

set (sources
  GazeboContext.cc
)

add_library(communication ${sources})
target_link_libraries(communication ${GAZEBO_LIBRARIES})

