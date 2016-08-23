cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set (sources
  GazeboRealSenseCamera.cc
)

set (headers
  GazeboRealSenseCamera.hh
)

add_library(sensors ${sources})
target_link_libraries(sensors communication ${GAZEBO_LIBRARIES} ${LDFLAGS})

