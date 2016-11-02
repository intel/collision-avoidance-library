cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set (sources
  DepthCamera.cc
  GazeboRealSenseCamera.cc
)

set (headers
  DepthCamera.hh
  GazeboRealSenseCamera.hh
)

add_library(sensors ${sources})
target_link_libraries(sensors communication ${GAZEBO_LIBRARIES} ${LDFLAGS})

